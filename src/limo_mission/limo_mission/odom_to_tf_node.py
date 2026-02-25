#!/usr/bin/env python3
"""
odom_to_tf_node — Publishes TF odom → base_footprint from /odom.

In simulation, the ros_gz_bridge may not populate
frame_id/child_frame_id correctly in /tf (issue #410), causing odometry to
not align with the map. This node takes /odom (which is usually correct) and
publishes the odom → base_footprint transform so that SLAM and RViz
display the robot's position correctly.

Usage: launched automatically by sim_slam.launch.py; or manually:
  ros2 run limo_mission odom_to_tf_node --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__("odom_to_tf_node")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("publish_tf", True)
        # If Gazebo uses a prefixed frame (e.g. limo_manipulator/base_footprint),
        # force base_footprint so SLAM and RViz match the URDF.
        self.declare_parameter("child_frame_id", "base_footprint")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self.odom_cb,
            10,
        )
        self.get_logger().info(
            "Publishing TF from /odom (odom -> base_footprint)"
        )

    def odom_cb(self, msg: Odometry):
        if not self.get_parameter("publish_tf").get_parameter_value().bool_value:
            return
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
