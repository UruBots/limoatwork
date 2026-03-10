#!/usr/bin/env python3
"""
manipulation_manager.py
========================
ROS 2 node that orchestrates pick-and-place for RoboCup @Work.

Works in BOTH Gazebo simulation (use_sim=True) and real robot (use_sim=False):
  - Simulation : connects to simulated arm_controller / gripper_controller
                 and the mock object_detector.
  - Real robot : same controllers (Dynamixel hardware) + RealSense detector.

The node is intentionally kept free of MoveIt2 dependencies so it compiles
and runs without the move_group server.  Arm motion is achieved by publishing
pre-computed JointTrajectory goals directly to the ROS 2 Control action servers.
MoveIt2 can be layered on top later for Cartesian / collision-aware planning.

Provides services
-----------------
  /manipulation/home   (std_srvs/Trigger) → move arm to safe home pose
  /manipulation/pick   (std_srvs/Trigger) → detect + grasp object at current WS
  /manipulation/place  (std_srvs/Trigger) → place carried object

Publishes
---------
  /manipulation/status (std_msgs/String) → "IDLE" | "PICKING" | "CARRYING" | "ERROR"

Parameters
----------
  use_sim          bool    True = simulation mode (default True)
  workspace        string  Current workspace name, e.g. "WS01"
  arm_poses_config string  Absolute path to arm_poses.yaml
  objects_config   string  Absolute path to objects_config.yaml
  detect_timeout   float   Max seconds to wait for detection service (default 10.0)
  motion_timeout   float   Max seconds per arm move (default 15.0)
  table_height_cm  int     Height of current workspace table in cm (default 10)
  placement_type   string  "standard"|"shelf"|"container"|"precise_placement"|"rotating_table"
  gz_world_name    string  Gazebo world name for teleport service (default "atwork_2025")

Typical call sequence from mission_manager
------------------------------------------
  1. Robot arrives at workspace (nav2 goal complete)
  2. Set 'workspace' parameter via /manipulation_manager/set_parameters
  3. Call /manipulation/pick  → arm picks the object
  4. Robot navigates to delivery zone
  5. Call /manipulation/place → arm releases the object
  6. Call /manipulation/home  → arm folds back for driving
"""

import json
import os
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

import yaml


class ManipulationManager(Node):
    """Orchestrates pick-and-place using direct joint trajectory control."""

    # Internal state machine states
    IDLE     = 'IDLE'
    PICKING  = 'PICKING'
    CARRYING = 'CARRYING'
    PLACING  = 'PLACING'
    ERROR    = 'ERROR'

    def __init__(self):
        super().__init__('manipulation_manager')

        # ------------------------------------------------------------------ #
        #  Parameters                                                         #
        # ------------------------------------------------------------------ #
        self.declare_parameter('use_sim', True)
        self.declare_parameter('workspace', 'WS01')
        self.declare_parameter('arm_poses_config', '')
        self.declare_parameter('objects_config', '')
        self.declare_parameter('detect_timeout', 10.0)
        self.declare_parameter('motion_timeout', 15.0)
        self.declare_parameter('table_height_cm', 10)
        self.declare_parameter('placement_type', 'standard')
        self.declare_parameter('gz_world_name', 'atwork_2025')

        self._use_sim = self.get_parameter('use_sim').value

        # ------------------------------------------------------------------ #
        #  Load YAML configs                                                  #
        # ------------------------------------------------------------------ #
        self._poses_cfg: dict = {}
        self._objects_cfg: dict = {}

        poses_path = self.get_parameter('arm_poses_config').value
        if poses_path and os.path.isfile(poses_path):
            with open(poses_path, 'r') as f:
                self._poses_cfg = yaml.safe_load(f)
            self.get_logger().info(f'Loaded arm poses: {poses_path}')
        else:
            self.get_logger().warn(
                f'arm_poses_config not found at "{poses_path}". '
                'Arm motion will be unavailable.'
            )

        obj_path = self.get_parameter('objects_config').value
        if obj_path and os.path.isfile(obj_path):
            with open(obj_path, 'r') as f:
                self._objects_cfg = yaml.safe_load(f)
            self.get_logger().info(f'Loaded objects config: {obj_path}')

        # ------------------------------------------------------------------ #
        #  Callback group                                                     #
        # ------------------------------------------------------------------ #
        # ReentrantCallbackGroup allows the action-client response callbacks
        # to fire while a service callback (pick/place/home) is blocking in
        # a polling loop.  Without this, rclpy.spin_until_future_complete()
        # deadlocks when called from inside a service callback in a
        # MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------ #
        #  ROS 2 Control action clients                                       #
        # ------------------------------------------------------------------ #
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

        # ------------------------------------------------------------------ #
        #  Detection service client                                           #
        # ------------------------------------------------------------------ #
        self._detect_client = self.create_client(
            Trigger, '/manipulation/detect_objects',
            callback_group=self._cb_group,
        )

        # ------------------------------------------------------------------ #
        #  State publisher                                                    #
        # ------------------------------------------------------------------ #
        self._status_pub = self.create_publisher(String, '/manipulation/status', 10)
        self._state = self.IDLE
        self._carried_object = None   # dict with detected object data

        # ------------------------------------------------------------------ #
        #  Service servers                                                    #
        # ------------------------------------------------------------------ #
        self.create_service(Trigger, '/manipulation/home',  self._home_cb,  callback_group=self._cb_group)
        self.create_service(Trigger, '/manipulation/pick',  self._pick_cb,  callback_group=self._cb_group)
        self.create_service(Trigger, '/manipulation/place', self._place_cb, callback_group=self._cb_group)

        self.get_logger().info(
            f'ManipulationManager ready '
            f'({"sim" if self._use_sim else "real"} mode).'
        )
        self._publish_status(self.IDLE)

    # ================================================================== #
    #  Service callbacks  (each runs in a separate thread)                #
    # ================================================================== #

    def _home_cb(self, _req, response):
        """Move arm to safe home/transport pose."""
        self.get_logger().info('[HOME] Moving to home pose...')
        ok = self._move_to_named_pose('home')
        response.success = ok
        response.message = 'Arm at home.' if ok else 'Home motion failed.'
        self._publish_status(self.IDLE if ok else self.ERROR)
        return response

    def _pick_cb(self, _req, response):
        """Full pick sequence: detect → pre-grasp → grasp → carry."""
        self._publish_status(self.PICKING)
        workspace = self.get_parameter('workspace').value
        height = self.get_parameter('table_height_cm').value
        placement_type = self.get_parameter('placement_type').value
        self.get_logger().info(
            f'[PICK] Starting pick at {workspace} '
            f'(height={height}cm, type={placement_type})...')

        # Select height/type-appropriate pose names with fallback
        if placement_type == 'rotating_table':
            scan_pose = self._get_pose_with_fallback(f'rt_scan', 'scan')
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

        # 1. Open gripper
        if not self._move_gripper('open'):
            return self._fail(response, 'Failed to open gripper.')

        # 2. Move arm to scan pose
        if not self._move_to_named_pose(scan_pose):
            return self._fail(response, f'Failed to reach {scan_pose} pose.')

        # 3. Detect objects
        detections = self._call_detect()
        if not detections:
            self.get_logger().warn('[PICK] No objects detected. Returning home.')
            self._move_to_named_pose('home')
            return self._fail(response, 'No objects detected.')

        # Use the first detected object in the list
        target = detections[0]
        self.get_logger().info(
            f"[PICK] Targeting: {target['id']} (type={target['type']}) "
            f"at ({target['x']:.3f}, {target['y']:.3f}, {target['z']:.3f})"
        )

        # 4. Pre-grasp pose (hover above object)
        if not self._move_to_named_pose(pre_grasp_pose):
            return self._fail(response, f'Failed to reach {pre_grasp_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        # 5. Grasp pose (lower to object)
        if not self._move_to_named_pose(grasp_pose):
            return self._fail(response, f'Failed to reach {grasp_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        # 6. Close gripper
        grip_pos = self._grasp_gripper_pos(target['type'])
        if not self._move_gripper_to(grip_pos):
            return self._fail(response, 'Failed to close gripper.')
        time.sleep(self._timing('settle_wait'))

        # 6b. [SIM] Teleport object to gripper position
        self._teleport_object_to_gripper(target['id'])

        # 7. Lift to carry pose
        if not self._move_to_named_pose('carry'):
            return self._fail(response, 'Failed to reach carry pose.')

        self._carried_object = target
        self._publish_status(self.CARRYING)
        response.success = True
        response.message = f"Picked {target['id']}."
        return response

    def _place_cb(self, _req, response):
        """Place sequence: place pose → release → retreat."""
        self._publish_status(self.PLACING)
        if self._carried_object is None:
            self.get_logger().warn('[PLACE] No object being carried.')
            response.success = False
            response.message = 'No object being carried.'
            return response

        placement_type = self.get_parameter('placement_type').value
        self.get_logger().info(
            f"[PLACE] Placing {self._carried_object['id']} "
            f"(type={placement_type})..."
        )

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

        # 1. Move to placement pose
        if not self._move_to_named_pose(place_pose):
            return self._fail(response, f'Failed to reach {place_pose} pose.')
        time.sleep(self._timing('settle_wait'))

        # 2. Open gripper
        if not self._move_gripper('open'):
            return self._fail(response, 'Failed to open gripper for place.')
        time.sleep(self._timing('settle_wait'))

        # 2b. [SIM] Teleport object to delivery zone (START = 0.3, 0.3)
        self._teleport_object_to_ground(self._carried_object['id'], 0.3, 0.3)

        # 3. Retreat to home
        self._move_to_named_pose('home')

        self._carried_object = None
        self._publish_status(self.IDLE)
        response.success = True
        response.message = 'Object placed.'
        return response

    # ================================================================== #
    #  Low-level arm motion helpers                                       #
    # ================================================================== #

    def _move_to_named_pose(self, pose_name: str) -> bool:
        """Move arm to a pre-defined joint configuration from arm_poses.yaml."""
        poses = self._poses_cfg.get('poses', {})
        if pose_name not in poses:
            self.get_logger().error(f'Pose "{pose_name}" not in arm_poses.yaml.')
            return False
        joint_positions = poses[pose_name]
        duration_sec = self._timing('joint_move_duration')
        return self._send_arm_goal(joint_positions, duration_sec)

    def _move_gripper(self, state: str) -> bool:
        """Open or close the gripper by named state ('open', 'close', 'partial')."""
        gripper_cfg = self._poses_cfg.get('gripper', {})
        pos = gripper_cfg.get(state)
        if pos is None:
            self.get_logger().error(f'Gripper state "{state}" not in config.')
            return False
        return self._move_gripper_to(pos)

    def _move_gripper_to(self, position: float) -> bool:
        """Move gripper_left_joint to a specific position (metres)."""
        duration_sec = self._timing('gripper_move_duration')
        return self._send_gripper_goal(position, duration_sec)

    def _send_arm_goal(self, joint_positions: list, duration_sec: float) -> bool:
        """
        Send a FollowJointTrajectory goal to arm_controller.
        Blocks until the goal is complete or motion_timeout is reached.
        """
        timeout = self.get_parameter('motion_timeout').value

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller action server not available.')
            return False

        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in joint_positions]
        pt.velocities = [0.0] * 4
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._arm_client.send_goal_async(goal)
        # Poll instead of spin_until_future_complete to avoid executor deadlock
        # when called from inside a service callback (MultiThreadedExecutor).
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

        error_code = result_future.result().result.error_code
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Arm motion failed (code={error_code}).')
            return False

        return True

    def _send_gripper_goal(self, position: float, duration_sec: float) -> bool:
        """
        Send a FollowJointTrajectory goal to gripper_controller.
        Both gripper_left_joint and gripper_right_joint are mirrored.
        """
        timeout = self.get_parameter('motion_timeout').value

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('gripper_controller action server not available.')
            return False

        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_joint']

        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._gripper_client.send_goal_async(goal)
        # Poll instead of spin_until_future_complete to avoid executor deadlock.
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
            self.get_logger().error('Gripper motion timed out.')
            return False

        error_code = result_future.result().result.error_code
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Gripper motion failed (code={error_code}).')
            return False

        return True

    # ================================================================== #
    #  Detection helper                                                   #
    # ================================================================== #

    def _call_detect(self) -> list:
        """Call /manipulation/detect_objects and return parsed list."""
        detect_timeout = self.get_parameter('detect_timeout').value
        if not self._detect_client.wait_for_service(timeout_sec=detect_timeout):
            self.get_logger().error(
                '/manipulation/detect_objects service not available.')
            return []

        future = self._detect_client.call_async(Trigger.Request())
        # Poll instead of spin_until_future_complete to avoid executor deadlock.
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
            self.get_logger().error(f'Invalid JSON from detector: {result.message}')
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

    def _grasp_gripper_pos(self, obj_type: str) -> float:
        cfg = self._objects_cfg.get('grasp_gripper', {})
        return float(cfg.get(obj_type, cfg.get('default', -0.005)))

    def _publish_status(self, state: str):
        self._state = state
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)

    def _fail(self, response, message: str):
        self.get_logger().error(f'[MANIPULATION] {message}')
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
        The object will reappear when placed at the delivery zone.
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
                self.get_logger().warn(f'[SIM] Failed to hide {obj_id}: {result.stderr.decode()}')
        except Exception as e:
            self.get_logger().warn(f'[SIM] Teleport exception: {e}')

    _place_counter = 0  # Class variable to offset placed objects

    def _teleport_object_to_ground(self, obj_id: str, base_x: float, base_y: float):
        """
        In simulation, teleport the object to the ground at delivery position.
        Objects are placed with slight offsets to avoid stacking.
        """
        if not self._use_sim:
            return
        try:
            world_name = self.get_parameter('gz_world_name').value
            ManipulationManager._place_counter += 1
            offset = (ManipulationManager._place_counter - 1) * 0.1
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
                self.get_logger().info(f'[SIM] Placed {obj_id} at delivery zone ({x:.2f}, {y:.2f}).')
            else:
                self.get_logger().warn(f'[SIM] Failed to place {obj_id}: {result.stderr.decode()}')
        except Exception as e:
            self.get_logger().warn(f'[SIM] Place teleport exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationManager()

    # Use a MultiThreadedExecutor so service callbacks run concurrently
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
