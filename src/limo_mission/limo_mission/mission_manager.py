#!/usr/bin/env python3
import math
import yaml
import threading
import time
import statistics
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, BackUp
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        # =========================
        # Params
        # =========================
        default_wp = (
            get_package_share_directory("limo_mission")
            + "/config/waypoints.yaml"
        )
        self.declare_parameter("waypoints_yaml", default_wp)
        self.declare_parameter("route_name", "test_route")

        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)

        self.declare_parameter("nav_timeout_sec", 120.0)
        self.declare_parameter("nav2_startup_timeout_sec", 120.0)

        self.declare_parameter("backup_distance", 0.10)
        self.declare_parameter("backup_speed", 0.08)
        self.declare_parameter("backup_timeout_sec", 3.0)

        # -------- LiDAR-only wall docking --------
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("dock_target_distance", 0.05)  # 5 cm from wall
        self.declare_parameter("dock_tol_enter", 0.01)         # enter docked state
        self.declare_parameter("dock_tol_exit", 0.02)          # exit docked state (hysteresis)
        self.declare_parameter("dock_ang_tol_m", 0.015)        # |left-right| <= this value

        # angular cones (degrees)
        self.declare_parameter("dock_front_cone_deg", 3.0)      # +/- 3°
        self.declare_parameter("dock_side_center_deg", 10.0)    # 10°
        self.declare_parameter("dock_side_halfwidth_deg", 1.0)  # +/- 1°

        # controller gains and limits
        self.declare_parameter("dock_kp_lin", 0.4)
        self.declare_parameter("dock_kp_ang", 1.5)
        self.declare_parameter("dock_v_max", 0.015)
        self.declare_parameter("dock_w_max", 0.25)

        # robustness
        self.declare_parameter("dock_min_valid_points", 5)
        self.declare_parameter("dock_control_rate_hz", 20.0)
        self.declare_parameter("dock_timeout_sec", 30.0)

        # number of docking retries per workstation
        self.declare_parameter("dock_retries", 2)

        # -------- Manipulation --------
        # If True, calls /manipulation/pick after docking and /manipulation/place
        # after reaching the delivery zone. Requires manipulation_manager to be
        # running (via manipulation.launch.py).
        self.declare_parameter("enable_manipulation", False)
        self.declare_parameter("manipulation_timeout_sec", 60.0)

        # If True, skip the fine wall-docking step and call pick directly after
        # coarse Nav2 navigation.  Useful in simulation where there are no real
        # workstation walls for the LiDAR to dock against.
        self.declare_parameter("skip_docking", False)

        # Load waypoints
        self.route_name = self.get_parameter("route_name").value
        wp_path = self.get_parameter("waypoints_yaml").value

        self.frame_id, self.wp_map, self.route = self._load_waypoints_and_route(wp_path, self.route_name)
        if not self.route:
            raise RuntimeError(f"Route '{self.route_name}' is empty or does not exist in YAML.")

        self.get_logger().info(
            f"frame_id='{self.frame_id}', route='{self.route_name}' with {len(self.route)} WS: {self.route}"
        )

        # Callback group for thread-safe operations
        self.cb_group = ReentrantCallbackGroup()

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self.cb_group)
        self.backup_client = ActionClient(self, BackUp, "/backup", callback_group=self.cb_group)

        # Manipulation service clients (only used if enable_manipulation=True)
        self.pick_client  = self.create_client(Trigger, '/manipulation/pick',  callback_group=self.cb_group)
        self.place_client = self.create_client(Trigger, '/manipulation/place', callback_group=self.cb_group)
        self.home_client  = self.create_client(Trigger, '/manipulation/home',  callback_group=self.cb_group)

        # Publisher initial pose
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        # LaserScan + cmd_vel
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.latest_scan: Optional[LaserScan] = None
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_scan, callback_group=self.cb_group
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer to start mission after a brief delay
        self.start_timer = self.create_timer(1.0, self._on_start_timer, callback_group=self.cb_group)

    def _on_start_timer(self):
        self.start_timer.cancel()
        threading.Thread(target=self._run_mission, daemon=True).start()

    # -------------------------
    # Scan callback
    # -------------------------
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # -------------------------
    # YAML loader
    # -------------------------
    def _load_waypoints_and_route(self, path: str, route_name: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        frame_id = data.get("frame_id", "map")

        wp_raw: Dict[str, dict] = data.get("waypoints", {})
        wp_map: Dict[str, Waypoint] = {}
        for name, v in wp_raw.items():
            wp_map[name] = Waypoint(
                name=name,
                x=float(v["x"]),
                y=float(v["y"]),
                yaw=float(v["yaw"]),
            )

        routes = data.get("routes", {})
        route_list = routes.get(route_name, [])
        for ws_name in route_list:
            if ws_name not in wp_map:
                raise RuntimeError(f"WS '{ws_name}' in route '{route_name}' does not exist in waypoints.")

        return frame_id, wp_map, route_list

    # -------------------------
    # Pose helper
    # -------------------------
    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        # stamp 0 avoids rejection due to sim_time / clock desync
        ps.header.stamp.sec = 0
        ps.header.stamp.nanosec = 0
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(yaw)
        return ps

    # -------------------------
    # Initial Pose Helper
    # -------------------------
    def _publish_initial_pose(self):
        x = self.get_parameter("initial_pose_x").value
        y = self.get_parameter("initial_pose_y").value
        yaw = self.get_parameter("initial_pose_yaw").value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = yaw_to_quat(float(yaw))

        self.get_logger().info(f"Publishing initial pose: x={x}, y={y}, yaw={yaw}")
        self.initial_pose_pub.publish(msg)

    # -------------------------
    # Action helpers
    # -------------------------
    def _wait_server(self, client: ActionClient, name: str, timeout_sec: float = 30.0) -> bool:
        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f"Action server {name} not available.")
            return False
        return True

    def _send_goal_and_wait(self, client: ActionClient, goal_msg, timeout_sec: float) -> bool:
        send_future = client.send_goal_async(goal_msg)
        goal_response_timeout = 30.0
        t0 = time.time()
        while not send_future.done():
            if time.time() - t0 > goal_response_timeout:
                self.get_logger().error(f"Timeout ({goal_response_timeout}s) waiting for goal response.")
                return False
            time.sleep(0.1)

        gh = send_future.result()
        if gh is None:
            self.get_logger().error("GoalHandle not received (None).")
            return False

        if not gh.accepted:
            self.get_logger().error("Goal rejected (accepted=False).")
            return False

        result_future = gh.get_result_async()
        t0 = time.time()
        while not result_future.done():
            if time.time() - t0 > timeout_sec:
                self.get_logger().error("Timeout waiting for result. Cancelling...")
                gh.cancel_goal_async()
                return False
            time.sleep(0.1)

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    # -------------------------
    # Wall docking helpers
    # -------------------------
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

    def _get_window_indices(self, scan: LaserScan, a0: float, a1: float) -> Tuple[int, int]:
        i0 = self._angle_to_index(scan, a0)
        i1 = self._angle_to_index(scan, a1)
        return (i0, i1) if i0 <= i1 else (i1, i0)

    def _valid_ranges(self, scan: LaserScan, i0: int, i1: int) -> List[float]:
        vals = []
        rmin = scan.range_min
        rmax = scan.range_max
        for r in scan.ranges[i0 : i1 + 1]:
            if not math.isfinite(r):
                continue
            # discard readings below range_min and spurious values above range_max (e.g. 65.53)
            if r <= rmin:
                continue
            if r >= rmax:
                continue
            vals.append(float(r))
        return vals

    def _median_in_cone(self, scan: LaserScan, center_deg: float, half_deg: float, min_pts: int) -> Optional[float]:
        c = deg2rad(center_deg)
        h = deg2rad(half_deg)
        i0, i1 = self._get_window_indices(scan, c - h, c + h)
        vals = self._valid_ranges(scan, i0, i1)
        if len(vals) < min_pts:
            return None
        return float(statistics.median(vals))

    def _dock_to_wall(self) -> bool:
        """
        Fine wall docking: aligns using left-right balance and approaches
        until dock_target_distance is reached.
        Returns True if docked successfully, False on timeout or missing scan.
        """
        target = float(self.get_parameter("dock_target_distance").value)
        tol_enter = float(self.get_parameter("dock_tol_enter").value)
        tol_exit = float(self.get_parameter("dock_tol_exit").value)
        ang_tol_m = float(self.get_parameter("dock_ang_tol_m").value)

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

        self.get_logger().info(
            f"Wall docking started: target={target:.3f}m, scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}"
        )

        docked = False
        t0 = time.time()
        period = 1.0 / max(1.0, rate)

        while rclpy.ok():
            if time.time() - t0 > timeout:
                self._publish_stop()
                self.get_logger().warn("Docking timeout.")
                return False

            # ensure we have a recent scan
            scan = self.latest_scan
            if scan is None:
                self._publish_stop()
                time.sleep(0.1)
                continue

            # compute cone medians
            front = self._median_in_cone(scan, 0.0, front_cone, min_pts)
            left = self._median_in_cone(scan, +side_center, side_half, min_pts)
            right = self._median_in_cone(scan, -side_center, side_half, min_pts)

            if front is None or left is None or right is None:
                self._publish_stop()
                time.sleep(period)
                continue

            err_dist = front - target
            err_ang = left - right  # in metres

            # hysteresis
            if docked:
                if abs(err_dist) >= tol_exit:
                    docked = False
            else:
                if abs(err_dist) <= tol_enter and abs(err_ang) <= ang_tol_m:
                    docked = True

            if docked:
                self._publish_stop()
                self.get_logger().info(
                    f"DOCKED ✅ front={front:.3f} err={err_dist:.3f} left-right={err_ang:.3f}"
                )
                return True

            # control output
            w = clamp(kp_ang * err_ang, -w_max, w_max)

            if abs(err_ang) > ang_tol_m:
                v = 0.0  # align angularly first
            else:
                v = clamp(kp_lin * err_dist, 0.0, v_max)  # advance only if err_dist > 0

            # safety: do not advance if already past the target
            if err_dist < -tol_enter:
                v = 0.0

            self._publish_cmd(v, w)
            time.sleep(period)

    # -------------------------
    # Manipulation helpers
    # -------------------------
    def _call_manipulation_service(self, client, service_name: str) -> bool:
        """Call a manipulation Trigger service and return success."""
        manipulation_timeout = float(self.get_parameter("manipulation_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Manipulation service {service_name} not available.')
            return False
        future = client.call_async(Trigger.Request())
        # Spin until complete (blocking within this daemon thread)
        deadline = time.monotonic() + manipulation_timeout
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f'{service_name} timed out.')
                return False
            time.sleep(0.1)
        result = future.result()
        if not result.success:
            self.get_logger().warn(f'{service_name}: {result.message}')
        return result.success

    def _set_manipulation_workspace(self, ws_name: str):
        """Update manipulation_manager's 'workspace' parameter."""
        if not self.pick_client.wait_for_service(timeout_sec=1.0):
            return  # manipulation not running, ignore
        # Use parameter service to tell the detector which workspace we're at
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as RclParam, ParameterValue, ParameterType
        set_param_client = self.create_client(
            SetParameters, '/manipulation_manager/set_parameters')
        if not set_param_client.wait_for_service(timeout_sec=2.0):
            return
        req = SetParameters.Request()
        pval = ParameterValue()
        pval.type = ParameterType.PARAMETER_STRING
        pval.string_value = ws_name
        param = RclParam()
        param.name = 'workspace'
        param.value = pval
        req.parameters = [param]
        self.create_client(SetParameters, '/object_detector/set_parameters')
        det_client = self.create_client(
            SetParameters, '/object_detector/set_parameters')
        if det_client.wait_for_service(timeout_sec=2.0):
            future = det_client.call_async(req)
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)

    # -------------------------
    # TF wait helper
    # -------------------------
    def _wait_for_tf(self, target_frame: str, source_frame: str,
                     timeout_sec: float = 90.0) -> bool:
        """
        Block until the transform target_frame←source_frame is available in the
        TF tree, or until timeout_sec wall-clock seconds have elapsed.
        Returns True on success, False on timeout.
        """
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)
        t0 = time.time()
        self.get_logger().info(
            f'Waiting for TF: {source_frame} → {target_frame} '
            f'(timeout {timeout_sec:.0f}s) …'
        )
        while time.time() - t0 < timeout_sec:
            try:
                tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
                elapsed = time.time() - t0
                self.get_logger().info(
                    f'TF {source_frame} → {target_frame} available '
                    f'after {elapsed:.1f}s ✅'
                )
                return True
            except Exception:
                time.sleep(0.5)
        self.get_logger().error(
            f'TF {source_frame} → {target_frame} not available '
            f'after {timeout_sec:.0f}s — aborting.'
        )
        return False

    # -------------------------
    # Mission loop
    # -------------------------
    def _run_mission(self):
        self.get_logger().info("Waiting for stabilisation (2s) before first goal...")
        time.sleep(2.0)

        self._publish_initial_pose()
        # Wait for AMCL to process initial pose and publish map->odom TF
        # (wall-clock sleep so it's independent of sim_time speed)
        time.sleep(5.0)

        dock_retries = int(self.get_parameter("dock_retries").value)
        nav_timeout = float(self.get_parameter("nav_timeout_sec").value)

        backup_distance = float(self.get_parameter("backup_distance").value)
        backup_speed = float(self.get_parameter("backup_speed").value)
        backup_timeout = float(self.get_parameter("backup_timeout_sec").value)

        enable_manipulation = bool(self.get_parameter("enable_manipulation").value)
        skip_docking       = bool(self.get_parameter("skip_docking").value)

        startup_timeout = float(self.get_parameter("nav2_startup_timeout_sec").value)
        if not self._wait_server(self.nav_client, "/navigate_to_pose", timeout_sec=startup_timeout):
            return
        if not self._wait_server(self.backup_client, "/backup", timeout_sec=startup_timeout):
            return

        # Ensure Gazebo's DiffDrive has published odom→base_footprint before
        # sending any Nav2 goal — otherwise the first goal is always rejected.
        if not self._wait_for_tf('odom', 'base_footprint', timeout_sec=90.0):
            self.get_logger().error('odom TF unavailable — aborting mission.')
            return
        # Also wait for map→odom which AMCL publishes once localised
        if not self._wait_for_tf('map', 'odom', timeout_sec=60.0):
            self.get_logger().error('map→odom TF unavailable — aborting mission.')
            return

        for i, ws_name in enumerate(self.route, start=1):
            wp = self.wp_map[ws_name]
            self.get_logger().info(
                f"[{i}/{len(self.route)}] Navigating to {wp.name} (x={wp.x:.3f}, y={wp.y:.3f}, yaw={wp.yaw:.3f})"
            )

            # 1) NavigateToPose (coarse navigation)
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = self._pose_stamped(wp.x, wp.y, wp.yaw)

            if not self._send_goal_and_wait(self.nav_client, nav_goal, timeout_sec=nav_timeout):
                self.get_logger().error(f"{wp.name}: navigation failed. Skipping.")
                continue

            # 2) Fine wall docking (skipped in simulation via skip_docking=true)
            if skip_docking:
                self.get_logger().info(
                    f"{wp.name}: skip_docking=true — skipping wall docking.")
                dock_ok = True
            else:
                dock_ok = False
                for attempt in range(1, dock_retries + 1):
                    self.get_logger().info(
                        f"{wp.name}: docking attempt {attempt}/{dock_retries}")
                    if self._dock_to_wall():
                        dock_ok = True
                        break

            if not dock_ok:
                self.get_logger().error(f"{wp.name}: docking failed. Moving to next WS.")
                continue

            # 3) Manipulation: pick object at this workspace
            if enable_manipulation:
                self.get_logger().info(f"{wp.name}: starting pick...")
                self._set_manipulation_workspace(ws_name)
                pick_ok = self._call_manipulation_service(
                    self.pick_client, '/manipulation/pick')
                if pick_ok:
                    self.get_logger().info(f"{wp.name}: object picked ✅")
                else:
                    self.get_logger().warn(f"{wp.name}: pick failed (continuing route).")

            # 4) BackUp (undock)
            self.get_logger().info(f"{wp.name}: undocking (backup {backup_distance:.2f} m)")
            b = BackUp.Goal()
            b.target = Point(x=abs(backup_distance), y=0.0, z=0.0)
            b.speed = float(backup_speed)
            b.time_allowance.sec = int(backup_timeout)
            b.time_allowance.nanosec = 0

            if not self._send_goal_and_wait(self.backup_client, b, timeout_sec=backup_timeout + 5.0):
                self.get_logger().warn(f"{wp.name}: backup failed (continuing anyway).")

            self.get_logger().info(f"{wp.name}: OK ✅")

        # 5) After all WS: if carrying an object, place it
        if enable_manipulation:
            self.get_logger().info("Route complete: placing object in delivery zone...")
            self._call_manipulation_service(self.place_client, '/manipulation/place')
            self._call_manipulation_service(self.home_client,  '/manipulation/home')

        self.get_logger().info("Route finished ✅")


def main():
    rclpy.init()
    node = MissionManager()
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
