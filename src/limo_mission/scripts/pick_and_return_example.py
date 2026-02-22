#!/usr/bin/env python3
"""
pick_and_return_example.py
==========================
Example mission: pick obj_f20_1 → return to base → place;
                 pick obj_m20_1 → return to base → place.

Both objects are in WS01 (see objects_config.yaml).

Prerequisites:
  - Simulation already running:
      ros2 launch limo_mission simulation_mission.launch.py
  - Wait ~30 s for Nav2 and manipulation_manager to be ready.

Run:
  source install/setup.bash
  python3 src/limo_mission/scripts/pick_and_return_example.py

Or as a ROS 2 executable after build:
  ros2 run limo_mission pick_and_return_example
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


# ---------------------------------------------------------------------------
# Mission node
# ---------------------------------------------------------------------------

class PickAndReturnMission(Node):
    """
    Executes a two-object pick-and-return mission:
      WS01 → pick(obj_f20_1) → START → place → home
      WS01 → pick(obj_m20_1) → START → place → home
    """

    # Waypoints (must match config/waypoints-sim.yaml)
    WS01  = dict(x=2.40, y=0.40, yaw=1.57)
    START = dict(x=0.00, y=0.00, yaw=0.00)

    def __init__(self):
        super().__init__('pick_return_mission')

        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._pick_cli  = self.create_client(Trigger, '/manipulation/pick')
        self._place_cli = self.create_client(Trigger, '/manipulation/place')
        self._home_cli  = self.create_client(Trigger, '/manipulation/home')

        self._param_cli = self.create_client(
            SetParameters, '/manipulation_manager/set_parameters')

    # ------------------------------------------------------------------ #
    #  Navigation                                                          #
    # ------------------------------------------------------------------ #

    def navigate_to(self, x: float, y: float, yaw: float,
                    timeout_sec: float = 120.0) -> bool:
        """
        Send a NavigateToPose goal and block until it finishes or times out.
        Returns True on success.
        """
        self.get_logger().info(
            f'Navigating → ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)')

        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('NavigateToPose server not available.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        send_future = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future,
                                         timeout_sec=timeout_sec)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                          timeout_sec=timeout_sec)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            return True

        self.get_logger().error(f'Navigation failed (status={status}).')
        return False

    # ------------------------------------------------------------------ #
    #  Manipulation helpers                                                #
    # ------------------------------------------------------------------ #

    def set_workspace(self, ws: str):
        """
        Update the 'workspace' parameter in manipulation_manager so the
        object detector knows which workspace the robot is currently at.
        """
        if not self._param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'manipulation_manager parameter service not available — '
                'workspace not updated.')
            return

        req = SetParameters.Request()
        p = Parameter()
        p.name = 'workspace'
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=ws)
        req.parameters = [p]

        future = self._param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f'Workspace set to {ws}.')

    def call_trigger(self, client, label: str,
                     timeout_sec: float = 60.0) -> bool:
        """
        Call a std_srvs/Trigger service.  Returns True on success.
        """
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{label}: service not available.')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error(f'{label}: no response (timeout?).')
            return False

        ok = future.result().success
        msg = future.result().message
        self.get_logger().info(
            f'{label}: {"OK" if ok else "FAILED"}'
            + (f' — {msg}' if msg else ''))
        return ok

    def pick(self) -> bool:
        return self.call_trigger(self._pick_cli, '/manipulation/pick')

    def place(self) -> bool:
        return self.call_trigger(self._place_cli, '/manipulation/place')

    def home(self) -> bool:
        return self.call_trigger(self._home_cli, '/manipulation/home')

    # ------------------------------------------------------------------ #
    #  Mission                                                             #
    # ------------------------------------------------------------------ #

    def run(self):
        """
        Full two-object pick-and-return mission.

        Step sequence
        -------------
        1. Navigate to WS01
        2. Pick obj_f20_1   (manipulation_manager uses the first detected object)
        3. Navigate to START
        4. Place object
        5. Move arm to home (safe transport pose)
        6. Navigate to WS01 again
        7. Pick obj_m20_1   (second object in the workspace)
        8. Navigate to START
        9. Place object
        10. Move arm to home
        """

        # ---- Object 1: obj_f20_1 (F20×20 aluminium profile) --------- #
        self.get_logger().info('─' * 50)
        self.get_logger().info('STEP 1/5 — Navigate to WS01 (obj_f20_1)')
        if not self.navigate_to(**self.WS01):
            self.get_logger().error('Could not reach WS01. Aborting mission.')
            return

        self.get_logger().info('STEP 2/5 — Pick obj_f20_1')
        self.set_workspace('WS01')
        time.sleep(1.5)  # allow detector to refresh after arm movement stops
        if not self.pick():
            self.get_logger().warn('Pick failed — continuing to return base.')

        self.get_logger().info('STEP 3/5 — Navigate to START (base)')
        if not self.navigate_to(**self.START):
            self.get_logger().error('Could not reach START. Aborting mission.')
            return

        self.get_logger().info('STEP 4/5 — Place obj_f20_1 at base')
        self.place()

        self.get_logger().info('STEP 5/5 — Arm to home (transport pose)')
        self.home()

        self.get_logger().info('─' * 50)
        self.get_logger().info('Object 1 (obj_f20_1) delivered ✅')
        self.get_logger().info('─' * 50)

        # ---- Object 2: obj_m20_1 (M20 hex nut) ---------------------- #
        self.get_logger().info('STEP 6/10 — Navigate to WS01 (obj_m20_1)')
        if not self.navigate_to(**self.WS01):
            self.get_logger().error('Could not reach WS01. Aborting mission.')
            return

        self.get_logger().info('STEP 7/10 — Pick obj_m20_1')
        self.set_workspace('WS01')
        time.sleep(1.5)
        if not self.pick():
            self.get_logger().warn('Pick failed — continuing to return base.')

        self.get_logger().info('STEP 8/10 — Navigate to START (base)')
        if not self.navigate_to(**self.START):
            self.get_logger().error('Could not reach START. Aborting mission.')
            return

        self.get_logger().info('STEP 9/10 — Place obj_m20_1 at base')
        self.place()

        self.get_logger().info('STEP 10/10 — Arm to home (transport pose)')
        self.home()

        self.get_logger().info('─' * 50)
        self.get_logger().info('Object 2 (obj_m20_1) delivered ✅')
        self.get_logger().info('Mission complete! 🎯')
        self.get_logger().info('─' * 50)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PickAndReturnMission()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
