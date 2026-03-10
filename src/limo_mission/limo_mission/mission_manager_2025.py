#!/usr/bin/env python3
"""
Mission Manager 2025 – RoboCup@Work World Cup 2025, Salvador

Hierarchical state machine for autonomous industrial logistics.
Supports all RoboCup@Work tests: BMT, BTT1/2, ATT1/2, Final.

Architecture (designed for reuse in future competitions):
  - StateMachine ........... generic engine with enter/execute/exit hooks
  - MissionState ........... competition-specific state enum
  - ArmController .......... thin wrapper per manipulator arm
  - MissionManager2025 ..... ROS 2 node that wires everything together

Hardware assumed:
  - LIMO mobile platform with LiDAR (Nav2 navigation)
  - OpenManipulator-X arm   (/arm/open_manipulator/*)
  - MyCobot robotic arm     (/arm/mycobot/*)
"""

import json
import math
import yaml
import threading
import time
import statistics
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Tuple

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import (
    PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, Twist,
)
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, BackUp
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, PointCloud2, PointField
    import struct
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════════════
#  Data models (reusable across competitions)
# ═══════════════════════════════════════════════════════════════════════

class ServiceAreaType(Enum):
    WS = "WS"
    SH = "SH"
    RT = "RT"
    PP = "PP"
    START = "START"
    FINISH = "FINISH"


@dataclass
class ServiceArea:
    name: str
    sa_type: ServiceAreaType
    x: float
    y: float
    yaw: float
    table_height_cm: int = 10


@dataclass
class TransportTask:
    task_id: int
    object_id: int
    object_name: str
    source: str
    destination: str
    container_color: Optional[str] = None
    status: str = "pending"


@dataclass
class DetectedObject:
    """Object detected by the perception system."""
    obj_id: str
    obj_type: str
    x: float
    y: float
    z: float
    confidence: float = 0.0
    method: str = ""
    tag_id: Optional[int] = None
    yaw: float = 0.0


class ArmId(Enum):
    OPEN_MANIPULATOR = "open_manipulator"
    MYCOBOT = "mycobot"


class TapeType(Enum):
    """Floor tape types per RoboCup@Work rulebook Section 3.2.4."""
    RED_WHITE = "red_white"          # Virtual Wall  → Major Collision
    YELLOW_BLACK = "yellow_black"    # Virtual Obstacle → Tape Collision (penalty)
    GREEN = "green"                  # Markup tape (START/FINISH) → informational
    NONE = "none"


# HSV colour ranges used by the camera-based tape detector.
# OpenCV HSV: H ∈ [0,179], S ∈ [0,255], V ∈ [0,255].
TAPE_HSV_RANGES = {
    "red_low":  (np.array([0, 100, 80]),   np.array([10, 255, 255]))   if _CV2_AVAILABLE else (None, None),
    "red_high": (np.array([170, 100, 80]), np.array([179, 255, 255]))  if _CV2_AVAILABLE else (None, None),
    "white":    (np.array([0, 0, 180]),     np.array([179, 40, 255]))   if _CV2_AVAILABLE else (None, None),
    "yellow":   (np.array([20, 100, 100]), np.array([35, 255, 255]))   if _CV2_AVAILABLE else (None, None),
    "black":    (np.array([0, 0, 0]),       np.array([179, 255, 50]))   if _CV2_AVAILABLE else (None, None),
    "green":    (np.array([35, 80, 60]),    np.array([85, 255, 255]))   if _CV2_AVAILABLE else (None, None),
}


# ═══════════════════════════════════════════════════════════════════════
#  Generic state-machine engine (reusable)
# ═══════════════════════════════════════════════════════════════════════

class MissionState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    WAITING_FOR_TASK = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    APPROACHING = auto()
    PERCEIVING = auto()
    PICKING = auto()
    PLACING = auto()
    UNDOCKING = auto()
    TAPE_DETECTED = auto()
    REPLANNING = auto()
    NAVIGATING_TO_FINISH = auto()
    FINISHED = auto()
    ERROR_RECOVERY = auto()


class StateMachine:
    """Generic state machine with enter/execute/exit hooks."""

    def __init__(self, initial_state: MissionState, logger: Optional[Callable] = None):
        self._state = initial_state
        self._prev_state: Optional[MissionState] = None
        self._handlers: Dict[MissionState, Callable] = {}
        self._on_enter: Dict[MissionState, Callable] = {}
        self._on_exit: Dict[MissionState, Callable] = {}
        self._logger = logger

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def prev_state(self) -> Optional[MissionState]:
        return self._prev_state

    def register(self, state: MissionState, execute: Callable,
                 on_enter: Optional[Callable] = None,
                 on_exit: Optional[Callable] = None):
        self._handlers[state] = execute
        if on_enter:
            self._on_enter[state] = on_enter
        if on_exit:
            self._on_exit[state] = on_exit

    def transition(self, new_state: MissionState):
        if new_state == self._state:
            return
        if self._logger:
            self._logger(f"[SM] {self._state.name} -> {new_state.name}")

        exit_fn = self._on_exit.get(self._state)
        if exit_fn:
            exit_fn()

        self._prev_state = self._state
        self._state = new_state

        enter_fn = self._on_enter.get(new_state)
        if enter_fn:
            enter_fn()

    def tick(self):
        handler = self._handlers.get(self._state)
        if handler:
            handler()


# ═══════════════════════════════════════════════════════════════════════
#  Arm controller – thin service-client wrapper (one per physical arm)
# ═══════════════════════════════════════════════════════════════════════

class ArmController:
    """Service-client wrapper for a manipulator arm.

    Connects to a manipulation namespace that provides:
      {ns}/pick            (std_srvs/Trigger)
      {ns}/place           (std_srvs/Trigger)
      {ns}/home            (std_srvs/Trigger)
      {ns}/detect_objects   (std_srvs/Trigger)  – optional

    For the OpenManipulator X (Dynamixel) ns="/manipulation".
    For the MyCobot                       ns="/mycobot".
    """

    def __init__(self, node: Node, arm_id: ArmId, service_ns: str, cb_group):
        self.arm_id = arm_id
        self._node = node
        self._ns = service_ns

        self.pick_client = node.create_client(
            Trigger, f"{service_ns}/pick", callback_group=cb_group)
        self.place_client = node.create_client(
            Trigger, f"{service_ns}/place", callback_group=cb_group)
        self.home_client = node.create_client(
            Trigger, f"{service_ns}/home", callback_group=cb_group)
        self.detect_client = node.create_client(
            Trigger, f"{service_ns}/detect_objects", callback_group=cb_group)

    def is_available(self, timeout: float = 1.0) -> bool:
        return self.pick_client.wait_for_service(timeout_sec=timeout)

    def _call_service(self, client, label: str, timeout: float = 60.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(f"{label} not available")
            return False, f"{label} not available"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                self._node.get_logger().error(f"{label} timed out")
                return False, f"{label} timed out"
            time.sleep(0.1)
        result = future.result()
        if not result.success:
            self._node.get_logger().warn(f"{label}: {result.message}")
        return result.success, result.message

    def pick(self, timeout: float = 60.0) -> bool:
        ok, _ = self._call_service(self.pick_client, f"{self._ns}/pick", timeout)
        return ok

    def place(self, timeout: float = 60.0) -> bool:
        ok, _ = self._call_service(self.place_client, f"{self._ns}/place", timeout)
        return ok

    def home(self, timeout: float = 60.0) -> bool:
        ok, _ = self._call_service(self.home_client, f"{self._ns}/home", timeout)
        return ok

    def detect(self, timeout: float = 10.0) -> List[DetectedObject]:
        """Call the detection service and parse the JSON response."""
        ok, message = self._call_service(
            self.detect_client, f"{self._ns}/detect_objects", timeout)
        if not ok:
            return []
        try:
            raw = json.loads(message)
        except (json.JSONDecodeError, TypeError):
            return []
        objects: List[DetectedObject] = []
        for d in raw:
            objects.append(DetectedObject(
                obj_id=d.get("id", "unknown"),
                obj_type=d.get("type", "unknown"),
                x=float(d.get("x", 0.0)),
                y=float(d.get("y", 0.0)),
                z=float(d.get("z", 0.0)),
                confidence=float(d.get("confidence", 0.0)),
                method=d.get("method", ""),
                tag_id=d.get("tag_id"),
                yaw=float(d.get("yaw", 0.0)),
            ))
        return objects

    def set_workspace(self, node: Node, workspace: str, manager_node_name: str,
                      table_height_cm: int = 10, placement_type: str = "standard"):
        """Set workspace, table_height_cm, and placement_type on the manipulation manager."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as RclParam, ParameterValue, ParameterType

        params = []

        p_ws = RclParam()
        p_ws.name = "workspace"
        p_ws.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                     string_value=workspace)
        params.append(p_ws)

        p_ht = RclParam()
        p_ht.name = "table_height_cm"
        p_ht.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=table_height_cm)
        params.append(p_ht)

        p_pt = RclParam()
        p_pt.name = "placement_type"
        p_pt.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                     string_value=placement_type)
        params.append(p_pt)

        req = SetParameters.Request()
        req.parameters = params
        client = node.create_client(
            SetParameters, f"{manager_node_name}/set_parameters")
        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(req)
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)


# ═══════════════════════════════════════════════════════════════════════
#  Utilities
# ═══════════════════════════════════════════════════════════════════════

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def distance(a: ServiceArea, b: ServiceArea) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def greedy_route(areas: List[ServiceArea], start: ServiceArea) -> List[ServiceArea]:
    """Nearest-neighbour TSP heuristic for visit ordering."""
    remaining = list(areas)
    route: List[ServiceArea] = []
    current = start
    while remaining:
        nearest = min(remaining, key=lambda sa: distance(current, sa))
        route.append(nearest)
        remaining.remove(nearest)
        current = nearest
    return route


# ═══════════════════════════════════════════════════════════════════════
#  Mission Manager 2025 – ROS 2 Node
# ═══════════════════════════════════════════════════════════════════════

class MissionManager2025(Node):

    MAX_INVENTORY = 3  # RoboCup@Work rule: max 3 objects carried

    def __init__(self):
        super().__init__("mission_manager_2025")

        # ── Parameters ──
        pkg = get_package_share_directory("limo_mission")
        self.declare_parameter("waypoints_yaml", f"{pkg}/config/waypoints_2025.yaml")
        self.declare_parameter("task_yaml", "")

        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)

        self.declare_parameter("nav_timeout_sec", 120.0)
        self.declare_parameter("nav2_startup_timeout_sec", 120.0)
        self.declare_parameter("nav_retries", 3)

        self.declare_parameter("backup_distance", 0.15)
        self.declare_parameter("backup_speed", 0.10)
        self.declare_parameter("backup_timeout_sec", 5.0)

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("dock_target_distance", 0.05)
        self.declare_parameter("dock_tol_enter", 0.01)
        self.declare_parameter("dock_tol_exit", 0.02)
        self.declare_parameter("dock_ang_tol_m", 0.015)
        self.declare_parameter("dock_front_cone_deg", 3.0)
        self.declare_parameter("dock_side_center_deg", 10.0)
        self.declare_parameter("dock_side_halfwidth_deg", 1.0)
        self.declare_parameter("dock_kp_lin", 0.4)
        self.declare_parameter("dock_kp_ang", 1.5)
        self.declare_parameter("dock_v_max", 0.015)
        self.declare_parameter("dock_w_max", 0.25)
        self.declare_parameter("dock_min_valid_points", 5)
        self.declare_parameter("dock_control_rate_hz", 20.0)
        self.declare_parameter("dock_timeout_sec", 30.0)
        self.declare_parameter("dock_retries", 2)
        self.declare_parameter("skip_docking", False)

        self.declare_parameter("manipulation_timeout_sec", 60.0)
        self.declare_parameter("enable_manipulation", False)

        self.declare_parameter("primary_arm", "open_manipulator")
        self.declare_parameter("secondary_arm", "mycobot")
        self.declare_parameter("primary_arm_ns", "/manipulation")
        self.declare_parameter("secondary_arm_ns", "/mycobot")
        self.declare_parameter("primary_manager_node", "/manipulation_manager")
        self.declare_parameter("secondary_manager_node", "/mycobot_manager")
        self.declare_parameter("detect_timeout_sec", 10.0)

        # ── Competition test mode ──
        self.declare_parameter("test_mode", "")

        # ── Tape detection parameters ──
        self.declare_parameter("enable_tape_detection", True)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("tape_obstacle_topic", "/tape_obstacles")
        self.declare_parameter("tape_roi_top_ratio", 0.55)
        self.declare_parameter("tape_min_pixel_ratio", 0.02)
        self.declare_parameter("tape_danger_pixel_ratio", 0.08)
        self.declare_parameter("tape_cooldown_sec", 3.0)

        # ── Callback group ──
        self.cb_group = ReentrantCallbackGroup()

        # ── Load arena config ──
        wp_path = self.get_parameter("waypoints_yaml").value
        self.frame_id, self.service_areas = self._load_service_areas(wp_path)
        self._info(f"Loaded {len(self.service_areas)} service areas from {wp_path}")

        # ── Task list & inventory ──
        self.tasks: List[TransportTask] = []
        self.inventory: List[TransportTask] = []
        self.visit_plan: List[Tuple[str, str]] = []  # (sa_name, action)
        self.plan_idx = 0
        self.current_sa: Optional[str] = None
        self.current_action: Optional[str] = None

        # ── Nav2 action clients ──
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self.cb_group)
        self.backup_client = ActionClient(
            self, BackUp, "/backup", callback_group=self.cb_group)

        # ── Arm controllers ──
        primary_id = ArmId(self.get_parameter("primary_arm").value)
        secondary_id = ArmId(self.get_parameter("secondary_arm").value)
        primary_ns = self.get_parameter("primary_arm_ns").value
        secondary_ns = self.get_parameter("secondary_arm_ns").value
        self.arms: Dict[ArmId, ArmController] = {
            primary_id: ArmController(self, primary_id, primary_ns, self.cb_group),
            secondary_id: ArmController(self, secondary_id, secondary_ns, self.cb_group),
        }
        self.primary_arm_id = primary_id
        self.secondary_arm_id = secondary_id
        self._primary_manager_node = self.get_parameter("primary_manager_node").value
        self._secondary_manager_node = self.get_parameter("secondary_manager_node").value

        # ── Publishers / Subscribers ──
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        scan_topic = self.get_parameter("scan_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.latest_scan: Optional[LaserScan] = None
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos_scan, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── Camera-based tape detection ──
        self._tape_type = TapeType.NONE
        self._tape_intensity = 0.0
        self._tape_last_trigger = 0.0
        self._tape_nav_goal_handle = None
        if _CV2_AVAILABLE and bool(self.get_parameter("enable_tape_detection").value):
            self._cv_bridge = CvBridge()
            cam_topic = self.get_parameter("camera_topic").value
            tape_obs_topic = self.get_parameter("tape_obstacle_topic").value
            self.tape_obstacle_pub = self.create_publisher(PointCloud2, tape_obs_topic, 10)
            self.camera_sub = self.create_subscription(
                Image, cam_topic, self._camera_cb, 5, callback_group=self.cb_group)
            self._info(f"Tape detection enabled on {cam_topic}")
        else:
            self.tape_obstacle_pub = None
            self.camera_sub = None
            if not _CV2_AVAILABLE:
                self._warn("cv2/cv_bridge not found – tape detection disabled")

        # ── State machine ──
        self.sm = StateMachine(MissionState.IDLE, logger=self._info)
        self._register_states()

        # ── Retry counters ──
        self._nav_attempt = 0
        self._dock_attempt = 0
        self._pick_attempt = 0

        # ── Perception state ──
        self.current_detections: List[DetectedObject] = []
        self.target_object: Optional[DetectedObject] = None

        # ── Start ──
        self.start_timer = self.create_timer(
            1.0, self._on_start, callback_group=self.cb_group)

    # ─────────────────────── Logging helpers ──────────────────────────

    _C = "\033[96m"
    _R = "\033[0m"

    def _info(self, msg: str):
        self.get_logger().info(f"{self._C}{msg}{self._R}")

    def _warn(self, msg: str):
        self.get_logger().warn(f"{self._C}{msg}{self._R}")

    def _error(self, msg: str):
        self.get_logger().error(f"{self._C}{msg}{self._R}")

    # ─────────────────────── Config loaders ───────────────────────────

    def _load_service_areas(self, path: str) -> Tuple[str, Dict[str, ServiceArea]]:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        frame_id = data.get("frame_id", "map")
        areas: Dict[str, ServiceArea] = {}
        for name, v in data.get("service_areas", {}).items():
            sa_type_str = v.get("type", "WS").upper()
            sa_type = ServiceAreaType[sa_type_str] if sa_type_str in ServiceAreaType.__members__ else ServiceAreaType.WS
            areas[name] = ServiceArea(
                name=name, sa_type=sa_type,
                x=float(v["x"]), y=float(v["y"]), yaw=float(v["yaw"]),
                table_height_cm=int(v.get("table_height_cm", 10)))
        return frame_id, areas

    def _load_tasks(self, path: str) -> List[TransportTask]:
        if not path:
            return []
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        tasks = []
        for i, t in enumerate(data.get("tasks", [])):
            tasks.append(TransportTask(
                task_id=i,
                object_id=int(t.get("object_id", 0)),
                object_name=str(t.get("object_name", "unknown")),
                source=str(t["source"]),
                destination=str(t["destination"]),
                container_color=t.get("container_color"),
            ))
        return tasks

    # ─────────────────── State machine registration ───────────────────

    def _register_states(self):
        self.sm.register(MissionState.IDLE, self._state_idle)
        self.sm.register(MissionState.INITIALIZING, self._state_init)
        self.sm.register(MissionState.WAITING_FOR_TASK, self._state_wait_task)
        self.sm.register(MissionState.PLANNING, self._state_planning)
        self.sm.register(MissionState.NAVIGATING, self._state_navigating)
        self.sm.register(MissionState.APPROACHING, self._state_approaching)
        self.sm.register(MissionState.PERCEIVING, self._state_perceiving)
        self.sm.register(MissionState.PICKING, self._state_picking)
        self.sm.register(MissionState.PLACING, self._state_placing)
        self.sm.register(MissionState.UNDOCKING, self._state_undocking)
        self.sm.register(MissionState.TAPE_DETECTED, self._state_tape_detected)
        self.sm.register(MissionState.REPLANNING, self._state_replanning)
        self.sm.register(MissionState.NAVIGATING_TO_FINISH, self._state_nav_finish)
        self.sm.register(MissionState.FINISHED, self._state_finished)
        self.sm.register(MissionState.ERROR_RECOVERY, self._state_error_recovery)

    # ─────────────────── Boot sequence ────────────────────────────────

    def _on_start(self):
        self.start_timer.cancel()
        threading.Thread(target=self._mission_loop, daemon=True).start()

    def _mission_loop(self):
        self.sm.transition(MissionState.INITIALIZING)
        while rclpy.ok() and self.sm.state != MissionState.FINISHED:
            self.sm.tick()
            time.sleep(0.05)

    # ─────────────────── Scan callback ────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ─────────────────── Camera / tape detection ──────────────────────

    def _camera_cb(self, msg: Image):
        """Detect floor tapes in the bottom portion of the camera image."""
        if not _CV2_AVAILABLE:
            return
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        h, w = frame.shape[:2]
        roi_top = int(h * float(self.get_parameter("tape_roi_top_ratio").value))
        roi = frame[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_pixels = roi.shape[0] * roi.shape[1]

        tape, intensity = self._classify_tape(hsv, roi_pixels)
        self._tape_type = tape
        self._tape_intensity = intensity

        if tape != TapeType.NONE and self.tape_obstacle_pub is not None:
            self._publish_tape_as_pointcloud(tape, msg.header)

    def _classify_tape(self, hsv, total_pixels: int) -> Tuple[TapeType, float]:
        """Return (tape_type, fraction_of_roi) for the dominant tape colour."""
        min_ratio = float(self.get_parameter("tape_min_pixel_ratio").value)

        rl, rh = TAPE_HSV_RANGES["red_low"]
        rl2, rh2 = TAPE_HSV_RANGES["red_high"]
        wl, wh = TAPE_HSV_RANGES["white"]
        yl, yh = TAPE_HSV_RANGES["yellow"]
        bl, bh = TAPE_HSV_RANGES["black"]
        gl, gh = TAPE_HSV_RANGES["green"]

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, rl, rh), cv2.inRange(hsv, rl2, rh2))
        white_mask = cv2.inRange(hsv, wl, wh)
        yellow_mask = cv2.inRange(hsv, yl, yh)
        black_mask = cv2.inRange(hsv, bl, bh)
        green_mask = cv2.inRange(hsv, gl, gh)

        red_ratio = cv2.countNonZero(red_mask) / total_pixels
        white_ratio = cv2.countNonZero(white_mask) / total_pixels
        yellow_ratio = cv2.countNonZero(yellow_mask) / total_pixels
        black_ratio = cv2.countNonZero(black_mask) / total_pixels
        green_ratio = cv2.countNonZero(green_mask) / total_pixels

        rw_score = min(red_ratio, white_ratio)
        yb_score = min(yellow_ratio, black_ratio)

        if rw_score >= min_ratio and rw_score >= yb_score and rw_score >= green_ratio:
            return TapeType.RED_WHITE, rw_score
        if yb_score >= min_ratio and yb_score >= green_ratio:
            return TapeType.YELLOW_BLACK, yb_score
        if green_ratio >= min_ratio:
            return TapeType.GREEN, green_ratio

        return TapeType.NONE, 0.0

    def _publish_tape_as_pointcloud(self, tape: TapeType, header):
        """Publish virtual obstacle points for Nav2 costmap integration.

        Projects a row of points ~0.3 m ahead in base_link frame so the
        local costmap marks the tape region as occupied.
        """
        forward_dist = 0.30
        n_points = 5
        spread = 0.20

        points = []
        for i in range(n_points):
            y = -spread + (2 * spread * i / max(1, n_points - 1))
            z = 0.0
            if tape == TapeType.RED_WHITE:
                z = 0.5  # tall obstacle ⇒ lethal
            elif tape == TapeType.YELLOW_BLACK:
                z = 0.2
            points.append((forward_dist, y, z))

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        buf = bytearray()
        for px, py, pz in points:
            buf += struct.pack('fff', px, py, pz)

        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.data = bytes(buf)
        cloud.is_dense = True
        self.tape_obstacle_pub.publish(cloud)

    # ═══════════════════════════════════════════════════════════════════
    #  State handlers
    # ═══════════════════════════════════════════════════════════════════

    def _state_idle(self):
        time.sleep(0.5)

    # ────────── INITIALIZING ──────────

    def _state_init(self):
        self._info("Initializing: publishing initial pose and waiting for Nav2...")
        time.sleep(2.0)
        self._publish_initial_pose()
        time.sleep(5.0)

        startup_timeout = float(self.get_parameter("nav2_startup_timeout_sec").value)
        if not self._wait_server(self.nav_client, "/navigate_to_pose", startup_timeout):
            self._error("Nav2 not available, aborting.")
            self.sm.transition(MissionState.FINISHED)
            return
        if not self._wait_server(self.backup_client, "/backup", startup_timeout):
            self._error("BackUp server not available, aborting.")
            self.sm.transition(MissionState.FINISHED)
            return

        if not self._wait_for_tf("odom", "base_footprint", 90.0):
            self._error("odom TF unavailable, aborting.")
            self.sm.transition(MissionState.FINISHED)
            return
        if not self._wait_for_tf("map", "odom", startup_timeout):
            self._error("map->odom TF unavailable, aborting.")
            self.sm.transition(MissionState.FINISHED)
            return

        self._info("Initialization complete.")
        self.sm.transition(MissionState.WAITING_FOR_TASK)

    # ────────── WAITING_FOR_TASK ──────────

    def _state_wait_task(self):
        test_spec = self._get_test_mode_spec()
        if test_spec:
            mode = self.get_parameter("test_mode").value.upper()
            self._info(f"Test mode: {mode} — {test_spec['description']}")
            self._info(f"  Objects: {test_spec['num_objects']}, "
                       f"Service areas: {test_spec['num_service_areas']}, "
                       f"Run time: {test_spec['run_time_sec']}s")
            if test_spec["includes_shelf"]:
                self._info("  Includes: shelves")
            if test_spec["includes_rt"]:
                self._info("  Includes: rotating table")
            if test_spec["includes_pp"]:
                self._info("  Includes: precise placement")
            if test_spec["includes_containers"]:
                self._info("  Includes: containers")

        task_yaml = self.get_parameter("task_yaml").value
        if task_yaml:
            self.tasks = self._load_tasks(task_yaml)
            self._info(f"Loaded {len(self.tasks)} transport tasks from YAML")
        else:
            self._build_default_patrol_tasks()

        if not self.tasks:
            self._warn("No tasks to execute. Going to FINISH.")
            self.sm.transition(MissionState.NAVIGATING_TO_FINISH)
            return

        self.sm.transition(MissionState.PLANNING)

    def _build_default_patrol_tasks(self):
        """Fallback: visit all WS-type service areas with a pick+place cycle."""
        ws_areas = [sa for sa in self.service_areas.values()
                    if sa.sa_type == ServiceAreaType.WS]
        finish_areas = [sa for sa in self.service_areas.values()
                        if sa.sa_type == ServiceAreaType.FINISH]
        target = finish_areas[0].name if finish_areas else (ws_areas[0].name if ws_areas else None)
        if not target:
            return
        for i, sa in enumerate(ws_areas):
            self.tasks.append(TransportTask(
                task_id=i, object_id=i, object_name=f"object_{i}",
                source=sa.name, destination=target))
        self._info(f"Built default patrol with {len(self.tasks)} tasks")

    # ────────── PLANNING ──────────

    def _state_planning(self):
        self._info("Planning route...")
        self.visit_plan = self._plan_visit_sequence()
        self.plan_idx = 0
        if not self.visit_plan:
            self._warn("Empty visit plan.")
            self.sm.transition(MissionState.NAVIGATING_TO_FINISH)
            return
        self._info(f"Visit plan ({len(self.visit_plan)} stops): "
                   f"{[(sa, act) for sa, act in self.visit_plan]}")
        self.sm.transition(MissionState.NAVIGATING)

    def _plan_visit_sequence(self) -> List[Tuple[str, str]]:
        """
        Build an ordered visit list: (service_area_name, action).
        action is "pick" or "place".

        Strategy: group pick tasks, respect inventory limit (3), then deliver.
        Uses greedy nearest-neighbor for ordering within each batch.
        """
        pending = [t for t in self.tasks if t.status == "pending"]
        if not pending:
            return []

        start_sa = None
        for sa in self.service_areas.values():
            if sa.sa_type == ServiceAreaType.START:
                start_sa = sa
                break
        if start_sa is None:
            first_task = pending[0]
            start_sa = self.service_areas.get(first_task.source, list(self.service_areas.values())[0])

        plan: List[Tuple[str, str]] = []
        remaining = list(pending)

        while remaining:
            batch_size = min(self.MAX_INVENTORY, len(remaining))
            batch = remaining[:batch_size]
            remaining = remaining[batch_size:]

            pick_areas = []
            for t in batch:
                sa = self.service_areas.get(t.source)
                if sa and sa not in pick_areas:
                    pick_areas.append(sa)

            ordered_pick = greedy_route(pick_areas, start_sa)
            for sa in ordered_pick:
                plan.append((sa.name, "pick"))

            place_areas = []
            for t in batch:
                sa = self.service_areas.get(t.destination)
                if sa and sa not in place_areas:
                    place_areas.append(sa)

            last_pick = ordered_pick[-1] if ordered_pick else start_sa
            ordered_place = greedy_route(place_areas, last_pick)
            for sa in ordered_place:
                plan.append((sa.name, "place"))

            if ordered_place:
                start_sa = ordered_place[-1]

        return plan

    # ────────── NAVIGATING ──────────

    def _state_navigating(self):
        if self.plan_idx >= len(self.visit_plan):
            self.sm.transition(MissionState.NAVIGATING_TO_FINISH)
            return

        sa_name, action = self.visit_plan[self.plan_idx]
        self.current_sa = sa_name
        self.current_action = action
        sa = self.service_areas.get(sa_name)
        if sa is None:
            self._error(f"Service area '{sa_name}' not found, skipping.")
            self.plan_idx += 1
            return

        self._info(f"[{self.plan_idx+1}/{len(self.visit_plan)}] "
                   f"Navigating to {sa_name} ({sa.sa_type.value}) for {action}")

        nav_ok = self._navigate_to_with_tape_check(sa)
        if nav_ok is None:
            return  # tape interrupt – state already transitioned
        if nav_ok:
            self._nav_attempt = 0
            skip_dock = bool(self.get_parameter("skip_docking").value)
            if skip_dock or sa.sa_type == ServiceAreaType.START:
                if action == "pick":
                    self.sm.transition(MissionState.PERCEIVING)
                else:
                    self.sm.transition(MissionState.PLACING)
            else:
                self.sm.transition(MissionState.APPROACHING)
        else:
            self._nav_attempt += 1
            max_retries = int(self.get_parameter("nav_retries").value)
            if self._nav_attempt >= max_retries:
                self._error(f"Navigation to {sa_name} failed after {max_retries} attempts, skipping.")
                self._mark_current_tasks_failed()
                self._nav_attempt = 0
                self.plan_idx += 1
            else:
                self._warn(f"Nav retry {self._nav_attempt}/{max_retries} for {sa_name}")
                self._clear_costmaps()
                time.sleep(1.0)

    # ────────── APPROACHING (wall docking) ──────────

    def _state_approaching(self):
        sa_name = self.current_sa
        max_dock = int(self.get_parameter("dock_retries").value)

        self._dock_attempt += 1
        self._info(f"{sa_name}: docking attempt {self._dock_attempt}/{max_dock}")

        if self._dock_to_wall():
            self._dock_attempt = 0
            if self.current_action == "pick":
                self.sm.transition(MissionState.PERCEIVING)
            else:
                self.sm.transition(MissionState.PLACING)
        elif self._dock_attempt >= max_dock:
            self._warn(f"{sa_name}: docking failed, proceeding anyway.")
            self._dock_attempt = 0
            if self.current_action == "pick":
                self.sm.transition(MissionState.PERCEIVING)
            else:
                self.sm.transition(MissionState.PLACING)

    # ────────── PERCEIVING ──────────

    def _state_perceiving(self):
        sa_name = self.current_sa or ""
        sa = self.service_areas.get(sa_name)
        arm = self._select_arm(sa)

        height = sa.table_height_cm if sa else 10
        ptype = self._sa_to_placement_type(sa, "pick")
        self._info(f"{sa_name}: perceiving objects with {arm.arm_id.value} "
                   f"(height={height}cm, type={ptype})...")

        manager_node = (self._primary_manager_node
                        if arm.arm_id == self.primary_arm_id
                        else self._secondary_manager_node)
        arm.set_workspace(self, sa_name, manager_node,
                          table_height_cm=height, placement_type=ptype)
        self._set_manipulation_workspace(sa_name)
        time.sleep(0.5)

        detect_timeout = float(self.get_parameter("detect_timeout_sec").value)
        self.current_detections = arm.detect(timeout=detect_timeout)

        if not self.current_detections:
            self._warn(f"{sa_name}: no objects detected, skipping pick.")
            self.target_object = None
            self.sm.transition(MissionState.UNDOCKING)
            return

        self._info(f"{sa_name}: detected {len(self.current_detections)} object(s): "
                   f"{[(d.obj_id, d.obj_type) for d in self.current_detections]}")

        self.target_object = self._match_detection_to_task(
            sa_name, self.current_detections)

        if self.target_object:
            self._info(f"{sa_name}: target → {self.target_object.obj_id} "
                       f"({self.target_object.obj_type})")
        else:
            self._info(f"{sa_name}: no task-matching object, picking first available")
            self.target_object = self.current_detections[0]

        self.sm.transition(MissionState.PICKING)

    # ────────── PICKING ──────────

    def _state_picking(self):
        sa_name = self.current_sa or ""
        enable_manip = bool(self.get_parameter("enable_manipulation").value)

        if not enable_manip:
            self._info(f"{sa_name}: manipulation disabled, simulating pick.")
            self._advance_tasks_picked(sa_name)
            self.sm.transition(MissionState.UNDOCKING)
            return

        sa = self.service_areas.get(sa_name)
        arm = self._select_arm(sa)
        target = self.target_object
        manip_timeout = float(self.get_parameter("manipulation_timeout_sec").value)

        obj_label = f"{target.obj_type}" if target else "unknown"
        self._info(f"{sa_name}: picking {obj_label} with {arm.arm_id.value} "
                   f"(ns={arm._ns})...")

        if arm.pick(manip_timeout):
            self._info(f"{sa_name}: pick successful ({obj_label})")
            self._advance_tasks_picked(sa_name)
            arm.home()
        else:
            self._pick_attempt += 1
            max_retries = 2
            if self._pick_attempt < max_retries:
                self._warn(f"{sa_name}: pick failed, retry {self._pick_attempt}/{max_retries}")
                arm.home()
                self.sm.transition(MissionState.PERCEIVING)
                return
            self._warn(f"{sa_name}: pick failed after {max_retries} attempts")
            self._pick_attempt = 0

        self._pick_attempt = 0
        self.sm.transition(MissionState.UNDOCKING)

    # ────────── PLACING ──────────

    def _state_placing(self):
        sa_name = self.current_sa or ""
        enable_manip = bool(self.get_parameter("enable_manipulation").value)

        if not enable_manip:
            self._info(f"{sa_name}: manipulation disabled, simulating place.")
            self._advance_tasks_placed(sa_name)
            self.sm.transition(MissionState.UNDOCKING)
            return

        sa = self.service_areas.get(sa_name)
        arm = self._select_arm(sa)
        manip_timeout = float(self.get_parameter("manipulation_timeout_sec").value)

        height = sa.table_height_cm if sa else 10
        ptype = self._sa_to_placement_type(sa, "place")
        placement_desc = self._describe_placement(sa)
        self._info(f"{sa_name}: placing with {arm.arm_id.value} "
                   f"({placement_desc}, height={height}cm, type={ptype})...")

        manager_node = (self._primary_manager_node
                        if arm.arm_id == self.primary_arm_id
                        else self._secondary_manager_node)
        arm.set_workspace(self, sa_name, manager_node,
                          table_height_cm=height, placement_type=ptype)

        if sa and sa.sa_type == ServiceAreaType.PP:
            self._info(f"{sa_name}: Precise Placement — detect cavities first")
            detect_timeout = float(self.get_parameter("detect_timeout_sec").value)
            pp_detections = arm.detect(timeout=detect_timeout)
            self._info(f"{sa_name}: PP cavities detected: {len(pp_detections)}")

        if ptype == "container":
            container_tasks = [t for t in self.inventory
                               if t.destination == sa_name
                               and t.container_color]
            for ct in container_tasks:
                self._info(f"{sa_name}: placing {ct.object_name} "
                           f"in {ct.container_color} container")

        if arm.place(manip_timeout):
            self._info(f"{sa_name}: place successful")
            self._advance_tasks_placed(sa_name)
        else:
            self._warn(f"{sa_name}: place failed")

        arm.home()
        self.sm.transition(MissionState.UNDOCKING)

    def _describe_placement(self, sa: Optional[ServiceArea]) -> str:
        if sa is None:
            return "standard"
        descs = {
            ServiceAreaType.WS: "workstation",
            ServiceAreaType.SH: "shelf (upper level)",
            ServiceAreaType.RT: "rotating table",
            ServiceAreaType.PP: "precise placement",
        }
        return descs.get(sa.sa_type, "standard")

    def _sa_to_placement_type(self, sa: Optional[ServiceArea],
                               action: str = "place") -> str:
        """Map service area type to manipulation placement_type parameter."""
        if sa is None:
            return "standard"
        type_map = {
            ServiceAreaType.SH: "shelf",
            ServiceAreaType.RT: "rotating_table",
            ServiceAreaType.PP: "precise_placement",
        }
        pt = type_map.get(sa.sa_type, "standard")
        # Check container placement: if placing and task has container_color
        if action == "place" and pt == "standard":
            for t in self.inventory:
                if t.destination == sa.name and t.container_color:
                    return "container"
        return pt

    # ────────── UNDOCKING ──────────

    def _state_undocking(self):
        sa_name = self.current_sa or ""
        dist = float(self.get_parameter("backup_distance").value)
        speed = float(self.get_parameter("backup_speed").value)
        timeout = float(self.get_parameter("backup_timeout_sec").value)

        self._info(f"{sa_name}: undocking (backup {dist:.2f} m)")
        goal = BackUp.Goal()
        goal.target = Point(x=abs(dist), y=0.0, z=0.0)
        goal.speed = speed
        goal.time_allowance.sec = int(timeout)

        self._send_goal_and_wait(self.backup_client, goal, timeout + 5.0)
        self._info(f"{sa_name}: done")

        self.plan_idx += 1
        if self.plan_idx >= len(self.visit_plan):
            self.sm.transition(MissionState.NAVIGATING_TO_FINISH)
        else:
            self.sm.transition(MissionState.NAVIGATING)

    # ────────── NAVIGATING_TO_FINISH ──────────

    def _state_nav_finish(self):
        finish_sa = None
        for sa in self.service_areas.values():
            if sa.sa_type == ServiceAreaType.FINISH:
                finish_sa = sa
                break

        if finish_sa is None:
            self._info("No FINISH area defined, ending mission.")
            self.sm.transition(MissionState.FINISHED)
            return

        self._info(f"Navigating to FINISH ({finish_sa.name})...")
        if self._navigate_to(finish_sa):
            self._info("Reached FINISH area.")
        else:
            self._warn("Failed to reach FINISH area.")

        self.sm.transition(MissionState.FINISHED)

    # ────────── FINISHED ──────────

    def _state_finished(self):
        delivered = sum(1 for t in self.tasks if t.status == "delivered")
        failed = sum(1 for t in self.tasks if t.status == "failed")
        score = self._estimate_score()
        self._info(f"Mission complete: {delivered} delivered, {failed} failed "
                   f"out of {len(self.tasks)} tasks. Estimated score: {score}")

    # ────────── TAPE_DETECTED ──────────

    def _state_tape_detected(self):
        tape = self._tape_type
        intensity = self._tape_intensity
        self._warn(f"Tape detected: {tape.value} (intensity {intensity:.3f})")

        self._publish_stop()
        time.sleep(0.3)

        if tape == TapeType.RED_WHITE:
            self._error("RED/WHITE tape = Virtual Wall! Stopping and replanning.")
            self.sm.transition(MissionState.REPLANNING)

        elif tape == TapeType.YELLOW_BLACK:
            self._warn("YELLOW/BLACK tape = Virtual Obstacle. Marking in costmap and replanning.")
            self.sm.transition(MissionState.REPLANNING)

        elif tape == TapeType.GREEN:
            self._info("GREEN tape = Markup (START/FINISH area). Continuing navigation.")
            self.sm.transition(MissionState.NAVIGATING)

        else:
            self.sm.transition(MissionState.NAVIGATING)

    # ────────── REPLANNING ──────────

    def _state_replanning(self):
        self._warn("Replanning: backing up, clearing costmaps, retrying navigation...")

        dist = float(self.get_parameter("backup_distance").value)
        speed = float(self.get_parameter("backup_speed").value)
        timeout = float(self.get_parameter("backup_timeout_sec").value)

        goal = BackUp.Goal()
        goal.target = Point(x=abs(dist), y=0.0, z=0.0)
        goal.speed = speed
        goal.time_allowance.sec = int(timeout)
        self._send_goal_and_wait(self.backup_client, goal, timeout + 5.0)

        self._clear_costmaps()
        time.sleep(1.0)
        self._tape_last_trigger = time.monotonic()
        self.sm.transition(MissionState.NAVIGATING)

    # ────────── ERROR_RECOVERY ──────────

    def _state_error_recovery(self):
        self._warn("Error recovery: clearing costmaps and retrying...")
        self._clear_costmaps()
        self._publish_stop()
        time.sleep(2.0)
        if self.sm.prev_state:
            self.sm.transition(self.sm.prev_state)
        else:
            self.sm.transition(MissionState.NAVIGATING)

    # ═══════════════════════════════════════════════════════════════════
    #  Arm selection strategy (override for custom logic)
    # ═══════════════════════════════════════════════════════════════════

    def _select_arm(self, sa: Optional[ServiceArea]) -> ArmController:
        """
        Select the best arm for the given service area type.
        Override this method to implement competition-specific strategies.

        Default strategy:
          - OpenManipulator X (Dynamixel, primary): standard WS/PP/START
          - MyCobot (secondary):  SH (shelves, needs reach under top shelf),
                                  RT (rotating table, benefits from wrist agility)
        Falls back to primary if secondary is not available.
        """
        primary = self.arms[self.primary_arm_id]
        secondary = self.arms[self.secondary_arm_id]

        if sa is None:
            return primary

        if sa.sa_type in (ServiceAreaType.SH, ServiceAreaType.RT):
            if secondary.is_available(timeout=0.5):
                self._info(f"Arm selection: {secondary.arm_id.value} "
                           f"for {sa.sa_type.value} ({sa.name})")
                return secondary
            self._warn(f"Secondary arm not available for {sa.sa_type.value}, "
                       f"using primary")

        return primary

    # ═══════════════════════════════════════════════════════════════════
    #  Task tracking helpers
    # ═══════════════════════════════════════════════════════════════════

    def _advance_tasks_picked(self, sa_name: str):
        for t in self.tasks:
            if t.source == sa_name and t.status == "pending":
                t.status = "picked"
                self.inventory.append(t)
                if len(self.inventory) >= self.MAX_INVENTORY:
                    break

    def _advance_tasks_placed(self, sa_name: str):
        placed = []
        for t in self.inventory:
            if t.destination == sa_name and t.status == "picked":
                t.status = "delivered"
                placed.append(t)
        for t in placed:
            self.inventory.remove(t)

    def _mark_current_tasks_failed(self):
        sa_name = self.current_sa
        action = self.current_action
        for t in self.tasks:
            if action == "pick" and t.source == sa_name and t.status == "pending":
                t.status = "failed"
            elif action == "place" and t.destination == sa_name and t.status == "picked":
                t.status = "failed"

    def _match_detection_to_task(self, sa_name: str,
                                 detections: List[DetectedObject]
                                 ) -> Optional[DetectedObject]:
        """Find the detection that matches a pending task at this service area."""
        pending_at_sa = [t for t in self.tasks
                         if t.source == sa_name and t.status == "pending"]
        if not pending_at_sa:
            return None

        wanted_names = {t.object_name.lower() for t in pending_at_sa}
        for det in detections:
            if det.obj_type.lower() in wanted_names:
                return det
            if det.obj_id.lower() in wanted_names:
                return det

        return detections[0] if detections else None

    # ═══════════════════════════════════════════════════════════════════
    #  Scoring estimation (Rulebook Chapter 6)
    # ═══════════════════════════════════════════════════════════════════

    def _estimate_score(self) -> int:
        """Estimate competition score per rulebook scoring table 6.1."""
        score = 0
        delivered = [t for t in self.tasks if t.status == "delivered"]
        picked = [t for t in self.tasks if t.status in ("picked", "delivered")]

        visited_areas: set = set()
        for t in picked:
            visited_areas.add(t.source)
        for t in delivered:
            visited_areas.add(t.destination)

        score += len(visited_areas) * 100  # Service area reached

        score += len(picked) * 100   # Object grasped
        score += len(delivered) * 100  # Object placed

        if len(delivered) == len(self.tasks) and len(self.tasks) > 0:
            score += len(delivered) * 10  # Perfect run bonus

        # FINISH area bonus
        finish_areas = [sa for sa in self.service_areas.values()
                        if sa.sa_type == ServiceAreaType.FINISH]
        if finish_areas:
            score += 100

        return score

    # ═══════════════════════════════════════════════════════════════════
    #  Competition test mode configuration (Rulebook Chapter 5)
    # ═══════════════════════════════════════════════════════════════════

    TEST_MODE_SPECS = {
        "BMT": {
            "description": "Basic Manipulation Test",
            "num_objects": 3,
            "num_service_areas": 2,
            "table_heights": [10],
            "includes_shelf": False,
            "includes_rt": False,
            "includes_pp": False,
            "includes_containers": False,
            "includes_obstacles": False,
            "includes_decoys": False,
            "includes_arbitrary_surface": False,
            "run_time_sec": 300,
        },
        "BTT1": {
            "description": "Basic Transportation Test 1",
            "num_objects": 4,
            "num_service_areas": 3,
            "table_heights": [10],
            "includes_shelf": False,
            "includes_rt": False,
            "includes_pp": False,
            "includes_containers": False,
            "includes_obstacles": False,
            "includes_decoys": False,
            "includes_arbitrary_surface": False,
            "run_time_sec": 300,
        },
        "BTT2": {
            "description": "Basic Transportation Test 2",
            "num_objects": 5,
            "num_service_areas": 4,
            "table_heights": [0, 5, 10, 15],
            "includes_shelf": False,
            "includes_rt": False,
            "includes_pp": False,
            "includes_containers": False,
            "includes_obstacles": True,
            "includes_decoys": True,
            "includes_arbitrary_surface": False,
            "run_time_sec": 300,
        },
        "ATT1": {
            "description": "Advanced Transportation Test 1",
            "num_objects": 6,
            "num_service_areas": 5,
            "table_heights": [0, 5, 10, 15],
            "includes_shelf": True,
            "includes_rt": False,
            "includes_pp": True,
            "includes_containers": False,
            "includes_obstacles": True,
            "includes_decoys": True,
            "includes_arbitrary_surface": True,
            "run_time_sec": 600,
        },
        "ATT2": {
            "description": "Advanced Transportation Test 2",
            "num_objects": 7,
            "num_service_areas": 6,
            "table_heights": [0, 5, 10, 15],
            "includes_shelf": True,
            "includes_rt": True,
            "includes_pp": False,
            "includes_containers": True,
            "includes_obstacles": True,
            "includes_decoys": True,
            "includes_arbitrary_surface": True,
            "run_time_sec": 600,
        },
        "FINAL": {
            "description": "Final Run",
            "num_objects": 10,
            "num_service_areas": 8,
            "table_heights": [0, 5, 10, 15],
            "includes_shelf": True,
            "includes_rt": True,
            "includes_pp": True,
            "includes_containers": True,
            "includes_obstacles": True,
            "includes_decoys": True,
            "includes_arbitrary_surface": True,
            "run_time_sec": 600,
        },
    }

    def _get_test_mode_spec(self) -> Optional[dict]:
        mode = self.get_parameter("test_mode").value.upper()
        if mode and mode in self.TEST_MODE_SPECS:
            return self.TEST_MODE_SPECS[mode]
        return None

    # ═══════════════════════════════════════════════════════════════════
    #  Navigation helpers
    # ═══════════════════════════════════════════════════════════════════

    def _navigate_to(self, sa: ServiceArea) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(sa.x, sa.y, sa.yaw)
        timeout = float(self.get_parameter("nav_timeout_sec").value)
        return self._send_goal_and_wait(self.nav_client, goal, timeout)

    def _navigate_to_with_tape_check(self, sa: ServiceArea) -> Optional[bool]:
        """Navigate while monitoring for dangerous floor tapes.

        Returns True/False like _navigate_to, or None if navigation was
        interrupted by a tape detection (state machine already transitioned).
        """
        if not bool(self.get_parameter("enable_tape_detection").value) or not _CV2_AVAILABLE:
            return self._navigate_to(sa)

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(sa.x, sa.y, sa.yaw)
        timeout = float(self.get_parameter("nav_timeout_sec").value)
        danger_ratio = float(self.get_parameter("tape_danger_pixel_ratio").value)
        cooldown = float(self.get_parameter("tape_cooldown_sec").value)

        send_future = self.nav_client.send_goal_async(goal)
        t0 = time.time()
        while not send_future.done():
            if time.time() - t0 > 30.0:
                self._error("Timeout waiting for goal acceptance.")
                return False
            time.sleep(0.1)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self._error("Nav goal rejected.")
            return False

        result_future = gh.get_result_async()
        while not result_future.done():
            if time.time() - t0 > timeout:
                self._error("Nav goal timeout, cancelling...")
                gh.cancel_goal_async()
                return False

            if (self._tape_type in (TapeType.RED_WHITE, TapeType.YELLOW_BLACK)
                    and self._tape_intensity >= danger_ratio
                    and (time.monotonic() - self._tape_last_trigger) > cooldown):
                self._warn(f"Tape interrupt: {self._tape_type.value} "
                           f"(intensity {self._tape_intensity:.3f})")
                gh.cancel_goal_async()
                time.sleep(0.5)
                self.sm.transition(MissionState.TAPE_DETECTED)
                return None

            time.sleep(0.1)

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp.sec = 0
        ps.header.stamp.nanosec = 0
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(yaw)
        return ps

    def _publish_initial_pose(self):
        x = float(self.get_parameter("initial_pose_x").value)
        y = float(self.get_parameter("initial_pose_y").value)
        yaw = float(self.get_parameter("initial_pose_yaw").value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05
        self._info(f"Publishing initial pose: x={x}, y={y}, yaw={yaw}")
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.5)

    def _wait_server(self, client: ActionClient, name: str, timeout: float = 30.0) -> bool:
        if not client.wait_for_server(timeout_sec=timeout):
            self._error(f"Action server {name} not available.")
            return False
        return True

    def _send_goal_and_wait(self, client: ActionClient, goal_msg, timeout_sec: float) -> bool:
        send_future = client.send_goal_async(goal_msg)
        t0 = time.time()
        while not send_future.done():
            if time.time() - t0 > 30.0:
                self._error("Timeout waiting for goal response.")
                return False
            time.sleep(0.1)

        gh = send_future.result()
        if gh is None or not gh.accepted:
            self._error("Goal rejected.")
            return False

        result_future = gh.get_result_async()
        t0 = time.time()
        while not result_future.done():
            if time.time() - t0 > timeout_sec:
                self._error("Goal timeout, cancelling...")
                gh.cancel_goal_async()
                return False
            time.sleep(0.1)

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _clear_costmaps(self):
        from std_srvs.srv import Empty
        for name in ["/local_costmap/clear_entirely_local_costmap",
                     "/global_costmap/clear_entirely_global_costmap"]:
            client = self.create_client(Empty, name)
            if client.wait_for_service(timeout_sec=1.0):
                client.call_async(Empty.Request())

    def _set_manipulation_workspace(self, ws_name: str):
        """Set the workspace parameter on the object_detector node."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as RclParam, ParameterValue, ParameterType
        req = SetParameters.Request()
        pval = ParameterValue()
        pval.type = ParameterType.PARAMETER_STRING
        pval.string_value = ws_name
        param = RclParam()
        param.name = "workspace"
        param.value = pval
        req.parameters = [param]
        for node_name in ["/object_detector"]:
            client = self.create_client(SetParameters, f"{node_name}/set_parameters")
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(req)
                deadline = time.monotonic() + 3.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)

    # ═══════════════════════════════════════════════════════════════════
    #  TF helper
    # ═══════════════════════════════════════════════════════════════════

    def _wait_for_tf(self, target: str, source: str, timeout: float = 90.0) -> bool:
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, self)
        t0 = time.time()
        self._info(f"Waiting for TF: {source} -> {target} (timeout {timeout:.0f}s)...")
        while time.time() - t0 < timeout:
            try:
                tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                self._info(f"TF {source} -> {target} available after {time.time()-t0:.1f}s")
                return True
            except Exception:
                time.sleep(0.5)
        self._error(f"TF {source} -> {target} not available after {timeout:.0f}s")
        return False

    # ═══════════════════════════════════════════════════════════════════
    #  Wall docking (LiDAR-based)
    # ═══════════════════════════════════════════════════════════════════

    def _publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    def _angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        a = clamp(angle_rad, scan.angle_min, scan.angle_max)
        idx = int(round((a - scan.angle_min) / scan.angle_increment))
        return int(clamp(idx, 0, len(scan.ranges) - 1))

    def _valid_ranges(self, scan: LaserScan, i0: int, i1: int) -> List[float]:
        return [
            float(r) for r in scan.ranges[i0:i1 + 1]
            if math.isfinite(r) and scan.range_min < r < scan.range_max
        ]

    def _median_in_cone(self, scan: LaserScan, center_deg: float,
                        half_deg: float, min_pts: int) -> Optional[float]:
        c = math.radians(center_deg)
        h = math.radians(half_deg)
        i0 = self._angle_to_index(scan, c - h)
        i1 = self._angle_to_index(scan, c + h)
        if i0 > i1:
            i0, i1 = i1, i0
        vals = self._valid_ranges(scan, i0, i1)
        if len(vals) < min_pts:
            return None
        return float(statistics.median(vals))

    def _dock_to_wall(self) -> bool:
        target = float(self.get_parameter("dock_target_distance").value)
        tol_enter = float(self.get_parameter("dock_tol_enter").value)
        tol_exit = float(self.get_parameter("dock_tol_exit").value)
        ang_tol = float(self.get_parameter("dock_ang_tol_m").value)
        front_cone = float(self.get_parameter("dock_front_cone_deg").value)
        side_center = float(self.get_parameter("dock_side_center_deg").value)
        side_half = float(self.get_parameter("dock_side_halfwidth_deg").value)
        kp_lin = float(self.get_parameter("dock_kp_lin").value)
        kp_ang = float(self.get_parameter("dock_kp_ang").value)
        v_max = float(self.get_parameter("dock_v_max").value)
        w_max = float(self.get_parameter("dock_w_max").value)
        min_pts = int(self.get_parameter("dock_min_valid_points").value)
        rate = float(self.get_parameter("dock_control_rate_hz").value)
        timeout = float(self.get_parameter("dock_timeout_sec").value)

        docked = False
        t0 = time.time()
        period = 1.0 / max(1.0, rate)

        while rclpy.ok():
            if time.time() - t0 > timeout:
                self._publish_stop()
                return False

            scan = self.latest_scan
            if scan is None:
                self._publish_stop()
                time.sleep(0.1)
                continue

            front = self._median_in_cone(scan, 0.0, front_cone, min_pts)
            left = self._median_in_cone(scan, +side_center, side_half, min_pts)
            right = self._median_in_cone(scan, -side_center, side_half, min_pts)

            if front is None or left is None or right is None:
                self._publish_stop()
                time.sleep(period)
                continue

            err_dist = front - target
            err_ang = left - right

            if docked:
                if abs(err_dist) >= tol_exit:
                    docked = False
            else:
                if abs(err_dist) <= tol_enter and abs(err_ang) <= ang_tol:
                    docked = True

            if docked:
                self._publish_stop()
                self._info(f"DOCKED: front={front:.3f} err={err_dist:.3f}")
                return True

            w = clamp(kp_ang * err_ang, -w_max, w_max)
            v = 0.0 if abs(err_ang) > ang_tol else clamp(kp_lin * err_dist, 0.0, v_max)
            if err_dist < -tol_enter:
                v = 0.0

            self._publish_cmd(v, w)
            time.sleep(period)

        return False


# ═══════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = MissionManager2025()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
