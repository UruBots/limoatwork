#!/usr/bin/env python3
"""
mycobot_manager.py
==================
ROS 2 node that orchestrates pick-and-place for the MyCobot 280 arm.

Mirrors the interface of manipulation_manager.py (OpenManipulator-X) but
targets the MyCobot 280 6-DOF arm.  Supports two control backends:

  use_pymycobot=True  (default, real robot)
    Uses the pymycobot Python library directly over serial (no ros2_control).
    Simpler setup, no hardware interface plugin needed.

  use_pymycobot=False  (simulation / ros2_control)
    Sends FollowJointTrajectory goals to a ros2_control JointTrajectoryController,
    same pattern as OpenManipulator-X.

Provides services
-----------------
  /mycobot/home   (std_srvs/Trigger) → move arm to safe home pose
  /mycobot/pick   (std_srvs/Trigger) → detect + grasp object at current WS
  /mycobot/place  (std_srvs/Trigger) → place carried object

Publishes
---------
  /mycobot/status (std_msgs/String) → "IDLE" | "PICKING" | "CARRYING" | "ERROR"
"""

import json
import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

import yaml

try:
    from pymycobot.mycobot import MyCobot
    _PYMYCOBOT_AVAILABLE = True
except ImportError:
    _PYMYCOBOT_AVAILABLE = False


class MycobotManager(Node):
    """Orchestrates pick-and-place for MyCobot 280."""

    IDLE     = 'IDLE'
    PICKING  = 'PICKING'
    CARRYING = 'CARRYING'
    PLACING  = 'PLACING'
    ERROR    = 'ERROR'

    def __init__(self):
        super().__init__('mycobot_manager')

        self.declare_parameter('use_sim', True)
        self.declare_parameter('use_pymycobot', True)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('workspace', 'WS01')
        self.declare_parameter('mycobot_poses_config', '')
        self.declare_parameter('objects_config', '')
        self.declare_parameter('detect_timeout', 10.0)
        self.declare_parameter('motion_timeout', 20.0)
        self.declare_parameter('table_height_cm', 10)
        self.declare_parameter('placement_type', 'standard')
        self.declare_parameter('gz_world_name', 'atwork_2025')

        self._use_sim = self.get_parameter('use_sim').value
        self._use_pymycobot = self.get_parameter('use_pymycobot').value

        self._poses_cfg: dict = {}
        self._objects_cfg: dict = {}

        poses_path = self.get_parameter('mycobot_poses_config').value
        if poses_path and os.path.isfile(poses_path):
            with open(poses_path, 'r') as f:
                self._poses_cfg = yaml.safe_load(f)
            self.get_logger().info(f'Loaded mycobot poses: {poses_path}')
        else:
            self.get_logger().warn(
                f'mycobot_poses_config not found at "{poses_path}".')

        obj_path = self.get_parameter('objects_config').value
        if obj_path and os.path.isfile(obj_path):
            with open(obj_path, 'r') as f:
                self._objects_cfg = yaml.safe_load(f)

        self._joint_names = self._poses_cfg.get('joint_names', [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
            'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
        ])
        self._gripper_joint_names = self._poses_cfg.get(
            'gripper_joint_names', ['gripper_joint'])
        self._n_joints = len(self._joint_names)

        self._cb_group = ReentrantCallbackGroup()

        # ── pymycobot direct connection ──
        self._mc = None
        if self._use_pymycobot and _PYMYCOBOT_AVAILABLE:
            port = self.get_parameter('port').value
            baud = int(self.get_parameter('baud').value)
            try:
                self._mc = MyCobot(port, baud)
                self.get_logger().info(
                    f'pymycobot connected: {port}@{baud}')
            except Exception as e:
                self.get_logger().error(f'pymycobot init failed: {e}')
                self._mc = None
        elif self._use_pymycobot and not _PYMYCOBOT_AVAILABLE:
            self.get_logger().warn(
                'pymycobot not installed — falling back to ros2_control')
            self._use_pymycobot = False

        # ── ros2_control action clients (fallback / simulation) ──
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            'mycobot_gripper_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

        # ── Detection service client ──
        self._detect_client = self.create_client(
            Trigger, '/mycobot/detect_objects',
            callback_group=self._cb_group,
        )

        # ── State publisher ──
        self._status_pub = self.create_publisher(String, '/mycobot/status', 10)
        self._state = self.IDLE
        self._carried_object = None

        # ── Service servers ──
        self.create_service(
            Trigger, '/mycobot/home', self._home_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '/mycobot/pick', self._pick_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '/mycobot/place', self._place_cb,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '/mycobot/detect_objects', self._detect_cb,
            callback_group=self._cb_group)

        mode = "pymycobot" if self._use_pymycobot else "ros2_control"
        env = "sim" if self._use_sim else "real"
        self.get_logger().info(
            f'MycobotManager ready ({env} mode, backend={mode}, '
            f'{self._n_joints} joints).')
        self._publish_status(self.IDLE)

    # ================================================================== #
    #  Service callbacks                                                  #
    # ================================================================== #

    def _home_cb(self, _req, response):
        self.get_logger().info('[HOME] Moving to home pose...')
        ok = self._move_to_named_pose('home')
        response.success = ok
        response.message = 'Arm at home.' if ok else 'Home motion failed.'
        self._publish_status(self.IDLE if ok else self.ERROR)
        return response

    def _pick_cb(self, _req, response):
        self._publish_status(self.PICKING)
        workspace = self.get_parameter('workspace').value
        height = self.get_parameter('table_height_cm').value
        placement_type = self.get_parameter('placement_type').value
        self.get_logger().info(
            f'[PICK] Starting pick at {workspace} '
            f'(height={height}cm, type={placement_type})...')

        # Select height/type-appropriate pose names with fallback
        if placement_type == 'rotating_table':
            scan_pose = self._get_pose_with_fallback('rt_scan', 'scan')
            pre_grasp_pose = self._get_pose_with_fallback('rt_pre_grasp', 'pre_grasp')
            grasp_pose = self._get_pose_with_fallback('rt_grasp', 'grasp')
        elif placement_type == 'shelf':
            scan_pose = self._get_pose_with_fallback(f'scan_{height}cm', 'scan')
            pre_grasp_pose = self._get_pose_with_fallback('shelf_pick', 'pre_grasp')
            grasp_pose = self._get_pose_with_fallback(f'grasp_{height}cm', 'grasp')
        else:
            scan_pose = self._get_pose_with_fallback(f'scan_{height}cm', 'scan')
            pre_grasp_pose = self._get_pose_with_fallback(f'pre_grasp_{height}cm', 'pre_grasp')
            grasp_pose = self._get_pose_with_fallback(f'grasp_{height}cm', 'grasp')

        if not self._move_gripper('open'):
            return self._fail(response, 'Failed to open gripper.')

        if not self._move_to_named_pose(scan_pose):
            return self._fail(response, f'Failed to reach {scan_pose} pose.')

        detections = self._call_detect()
        if not detections:
            self.get_logger().warn('[PICK] No objects detected.')
            self._move_to_named_pose('home')
            return self._fail(response, 'No objects detected.')

        target = detections[0]
        self.get_logger().info(
            f"[PICK] Target: {target.get('id', '?')} "
            f"(type={target.get('type', '?')})")

        if not self._move_to_named_pose(pre_grasp_pose):
            return self._fail(response, f'Failed to reach {pre_grasp_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        if not self._move_to_named_pose(grasp_pose):
            return self._fail(response, f'Failed to reach {grasp_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        grip = self._grasp_gripper_value(target.get('type', 'default'))
        if not self._move_gripper_to(grip):
            return self._fail(response, 'Failed to close gripper.')
        time.sleep(self._timing('settle_wait'))

        # [SIM] Teleport object to gripper position
        self._teleport_object_to_gripper(target.get('id', ''))

        if not self._move_to_named_pose('carry'):
            return self._fail(response, 'Failed to reach carry.')

        self._carried_object = target
        self._publish_status(self.CARRYING)
        response.success = True
        response.message = f"Picked {target.get('id', '?')}."
        return response

    def _place_cb(self, _req, response):
        self._publish_status(self.PLACING)
        if self._carried_object is None:
            response.success = False
            response.message = 'No object being carried.'
            return response

        placement_type = self.get_parameter('placement_type').value
        self.get_logger().info(
            f"[PLACE] Placing {self._carried_object.get('id', '?')} "
            f"(type={placement_type})...")

        # Select placement-type-appropriate pose
        place_poses = {
            'shelf': 'shelf_place',
            'container': 'container_place',
            'precise_placement': 'pp_place',
            'rotating_table': 'rt_place',
        }
        preferred = place_poses.get(placement_type)
        if preferred:
            place_pose = self._get_pose_with_fallback(preferred, 'place_ready')
        else:
            place_pose = 'place_ready'

        if not self._move_to_named_pose(place_pose):
            return self._fail(response, f'Failed to reach {place_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        if not self._move_gripper('open'):
            return self._fail(response, 'Failed to open gripper.')
        time.sleep(self._timing('settle_wait'))

        # [SIM] Teleport object to delivery zone
        self._teleport_object_to_ground(
            self._carried_object.get('id', ''), 0.3, 0.3)

        self._move_to_named_pose('home')
        self._carried_object = None
        self._publish_status(self.IDLE)
        response.success = True
        response.message = 'Object placed.'
        return response

    def _detect_cb(self, _req, response):
        """Proxy detection service — calls the shared object_detector."""
        detections = self._call_detect()
        response.success = len(detections) > 0
        response.message = json.dumps(detections)
        return response

    # ================================================================== #
    #  Arm motion — dual backend                                          #
    # ================================================================== #

    def _move_to_named_pose(self, pose_name: str) -> bool:
        poses = self._poses_cfg.get('poses', {})
        if pose_name not in poses:
            self.get_logger().error(f'Pose "{pose_name}" not in config.')
            return False
        joint_positions = poses[pose_name]
        if len(joint_positions) != self._n_joints:
            self.get_logger().error(
                f'Pose "{pose_name}" has {len(joint_positions)} values, '
                f'expected {self._n_joints}.')
            return False

        if self._use_pymycobot and self._mc is not None:
            return self._pymycobot_move(joint_positions)
        else:
            duration = self._timing('joint_move_duration')
            return self._ros2control_move(joint_positions, duration)

    def _move_gripper(self, state: str) -> bool:
        if self._use_pymycobot and self._mc is not None:
            cfg = self._poses_cfg.get('gripper_pymycobot', {})
            val = cfg.get(state)
            if val is None:
                return False
            return self._pymycobot_gripper(int(val))
        else:
            cfg = self._poses_cfg.get('gripper', {})
            pos = cfg.get(state)
            if pos is None:
                return False
            return self._ros2control_gripper(float(pos))

    def _move_gripper_to(self, value) -> bool:
        if self._use_pymycobot and self._mc is not None:
            return self._pymycobot_gripper(int(value))
        else:
            return self._ros2control_gripper(float(value))

    # ── pymycobot backend ──

    def _pymycobot_move(self, joint_positions_rad: list) -> bool:
        if self._mc is None:
            return False
        try:
            angles_deg = [round(math.degrees(r), 2) for r in joint_positions_rad]
            speed = int(self._timing('pymycobot_speed'))
            self._mc.send_angles(angles_deg, speed)
            move_duration = self._timing('joint_move_duration')
            time.sleep(move_duration)
            return True
        except Exception as e:
            self.get_logger().error(f'pymycobot move error: {e}')
            return False

    def _pymycobot_gripper(self, value: int) -> bool:
        if self._mc is None:
            return False
        try:
            self._mc.set_gripper_value(value, 50)
            time.sleep(self._timing('gripper_move_duration'))
            return True
        except Exception as e:
            self.get_logger().error(f'pymycobot gripper error: {e}')
            return False

    # ── ros2_control backend ──

    def _ros2control_move(self, joint_positions: list,
                          duration_sec: float) -> bool:
        timeout = self.get_parameter('motion_timeout').value
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('mycobot_arm_controller not available.')
            return False

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in joint_positions]
        pt.velocities = [0.0] * self._n_joints
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._arm_client.send_goal_async(goal)

        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or not future.result().accepted:
            self.get_logger().error('Arm goal rejected.')
            return False

        result_future = future.result().get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().error('Arm motion timed out.')
            return False

        err = result_future.result().result.error_code
        if err != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Arm motion failed (code={err}).')
            return False
        return True

    def _ros2control_gripper(self, position: float) -> bool:
        timeout = self.get_parameter('motion_timeout').value
        duration_sec = self._timing('gripper_move_duration')

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('mycobot_gripper_controller not available.')
            return False

        traj = JointTrajectory()
        traj.joint_names = list(self._gripper_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._gripper_client.send_goal_async(goal)

        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or not future.result().accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        result_future = future.result().get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().error('Gripper timed out.')
            return False

        err = result_future.result().result.error_code
        if err != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Gripper failed (code={err}).')
            return False
        return True

    # ================================================================== #
    #  Detection helper                                                   #
    # ================================================================== #

    def _call_detect(self) -> list:
        detect_timeout = self.get_parameter('detect_timeout').value
        if not self._detect_client.wait_for_service(timeout_sec=detect_timeout):
            self.get_logger().error('detect_objects service not available.')
            return []

        future = self._detect_client.call_async(Trigger.Request())
        deadline = time.monotonic() + detect_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().error('Detection timed out.')
            return []

        result = future.result()
        if not result.success:
            return []
        try:
            return json.loads(result.message)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON: {result.message}')
            return []

    # ================================================================== #
    #  Utility                                                            #
    # ================================================================== #

    def _get_pose_with_fallback(self, preferred: str, fallback: str) -> str:
        """Return *preferred* if it exists in the poses config, else *fallback*."""
        poses = self._poses_cfg.get('poses', {})
        if preferred in poses:
            self.get_logger().debug(f'Using pose "{preferred}".')
            return preferred
        self.get_logger().debug(
            f'Pose "{preferred}" not in config, falling back to "{fallback}".')
        return fallback

    def _timing(self, key: str) -> float:
        return float(self._poses_cfg.get('timing', {}).get(key, 1.0))

    def _grasp_gripper_value(self, obj_type: str):
        if self._use_pymycobot:
            return self._poses_cfg.get(
                'gripper_pymycobot', {}).get('close', 100)
        cfg = self._objects_cfg.get('grasp_gripper', {})
        return float(cfg.get(obj_type, cfg.get('default', -0.005)))

    def _publish_status(self, state: str):
        self._state = state
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)

    def _fail(self, response, message: str):
        self.get_logger().error(f'[MYCOBOT] {message}')
        response.success = False
        response.message = message
        self._publish_status(self.ERROR)
        return response

    # ================================================================== #
    #  Gazebo Object Teleportation (Simulation Only)                      #
    # ================================================================== #

    def _teleport_object_to_gripper(self, obj_id: str):
        """
        In simulation, "pick" the object by hiding it underground.
        Uses ign service to call /world/<world>/set_pose.
        """
        if not self._use_sim:
            return
        try:
            world_name = self.get_parameter('gz_world_name').value
            cmd = [
                'ign', 'service', '-s', f'/world/{world_name}/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                f'name: "{obj_id}", position: {{x: 0.0, y: 0.0, z: -5.0}}'
            ]
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(f'[SIM] Object {obj_id} picked (hidden).')
            else:
                self.get_logger().warn(
                    f'[SIM] Failed to hide {obj_id}: {result.stderr.decode()}')
        except Exception as e:
            self.get_logger().warn(f'[SIM] Teleport exception: {e}')

    _place_counter = 0

    def _teleport_object_to_ground(self, obj_id: str, base_x: float, base_y: float):
        """
        In simulation, teleport the object to the ground at delivery position.
        Objects are placed with slight offsets to avoid stacking.
        """
        if not self._use_sim:
            return
        try:
            world_name = self.get_parameter('gz_world_name').value
            MycobotManager._place_counter += 1
            offset = (MycobotManager._place_counter - 1) * 0.1
            x = base_x + offset
            y = base_y

            cmd = [
                'ign', 'service', '-s', f'/world/{world_name}/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                f'name: "{obj_id}", position: {{x: {x}, y: {y}, z: 0.05}}'
            ]
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(
                    f'[SIM] Placed {obj_id} at delivery zone ({x:.2f}, {y:.2f}).')
            else:
                self.get_logger().warn(
                    f'[SIM] Failed to place {obj_id}: {result.stderr.decode()}')
        except Exception as e:
            self.get_logger().warn(f'[SIM] Place teleport exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MycobotManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
