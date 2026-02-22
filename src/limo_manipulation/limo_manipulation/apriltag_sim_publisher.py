#!/usr/bin/env python3
"""
apriltag_sim_publisher.py
==========================
Simulation-only node that publishes synthetic AprilTagDetectionArray messages
as if a real apriltag_detector were running on the camera images.

WHY this exists
---------------
In Gazebo simulation the RGB camera sensor renders the scene, but
texture-mapped PNG AprilTags may not render with enough contrast for the
real apriltag_detector to decode them reliably (depends on rendering engine,
lighting, and camera resolution).  This node bypasses the image pipeline
entirely by reading the known object positions from `apriltag_config.yaml`
and, when the robot's camera is within detection range, publishing the
corresponding AprilTag detection with a realistic 3D pose in the camera frame.

This lets you develop and test the entire manipulation pipeline — AprilTag
detection → pose estimation → pick/place — in simulation without worrying
about Gazebo rendering quality.

On the real robot
-----------------
This node is NOT launched.  The real `apriltag_detector` runs on the
RealSense RGB stream instead (see manipulation.launch.py).

Published topics
----------------
  /apriltag/detections  (apriltag_msgs/msg/AprilTagDetectionArray)
    → same format as the real detector; consumed by object_detector.py

  /apriltag/detections_with_pose  (geometry_msgs/msg/PoseArray)
    → visualisation helper: one pose per detection in the 'map' frame

Subscribed topics
-----------------
  /tf, /tf_static  → to look up camera_color_optical_frame in map frame

Parameters
----------
  apriltag_config   string   Absolute path to apriltag_config.yaml
  objects_config    string   Absolute path to objects_config.yaml
  camera_frame      string   TF frame of the camera (default: camera_color_optical_frame)
  publish_rate_hz   float    How often to re-publish detections (default: 10.0)
  detection_range   float    Max distance camera→tag for fake detection (metres, default 1.2)
  fov_deg           float    Camera horizontal FOV for visibility check (default 60.0)
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point as TagPoint

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped

import yaml
import numpy as np


class AprilTagSimPublisher(Node):
    """Publishes synthetic AprilTag detections from known world positions."""

    def __init__(self):
        super().__init__('apriltag_sim_publisher')

        self.declare_parameter('apriltag_config', '')
        self.declare_parameter('objects_config',  '')
        self.declare_parameter('camera_frame',    'camera_color_optical_frame')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('detection_range', 1.2)
        self.declare_parameter('fov_deg',         60.0)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._max_range    = self.get_parameter('detection_range').value
        self._fov_rad      = math.radians(self.get_parameter('fov_deg').value)

        # ------------------------------------------------------------------ #
        #  Load configs                                                       #
        # ------------------------------------------------------------------ #
        self._tag_map: dict   = {}   # tag_id → tag config dict
        self._world_tags: list = []  # list of {id, x, y, z, yaw, object_type}

        ap_cfg_path  = self.get_parameter('apriltag_config').value
        obj_cfg_path = self.get_parameter('objects_config').value

        if ap_cfg_path and os.path.isfile(ap_cfg_path):
            with open(ap_cfg_path, 'r') as f:
                ap_cfg = yaml.safe_load(f)
            for tag in ap_cfg.get('tags', []):
                self._tag_map[tag['id']] = tag
            self.get_logger().info(
                f'Loaded {len(self._tag_map)} tag definitions from {ap_cfg_path}')
        else:
            self.get_logger().warn(f'apriltag_config not found at "{ap_cfg_path}".')

        if obj_cfg_path and os.path.isfile(obj_cfg_path):
            with open(obj_cfg_path, 'r') as f:
                obj_cfg = yaml.safe_load(f)
            sim_positions = obj_cfg.get('sim_object_positions', {})
            for obj_name, data in sim_positions.items():
                obj_type = data.get('type', '')
                # Find which tag ID corresponds to this object type
                for tag_id, tag_info in self._tag_map.items():
                    if tag_info.get('object_type') == obj_type:
                        pos = data['position']
                        self._world_tags.append({
                            'id':          tag_id,
                            'x':           pos[0],
                            'y':           pos[1],
                            'z':           pos[2],
                            'yaw':         data.get('yaw', 0.0),
                            'object_type': obj_type,
                            'name':        obj_name,
                        })
            self.get_logger().info(
                f'Loaded {len(self._world_tags)} world tag positions from {obj_cfg_path}')
        else:
            self.get_logger().warn(f'objects_config not found at "{obj_cfg_path}".')

        # ------------------------------------------------------------------ #
        #  TF2                                                                #
        # ------------------------------------------------------------------ #
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------ #
        #  Publishers                                                         #
        # ------------------------------------------------------------------ #
        self._det_pub  = self.create_publisher(
            AprilTagDetectionArray, '/apriltag/detections', 10)
        self._pose_pub = self.create_publisher(
            PoseArray, '/apriltag/detections_with_pose', 10)

        # ------------------------------------------------------------------ #
        #  Timer                                                              #
        # ------------------------------------------------------------------ #
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_cb)

        self.get_logger().info(
            f'AprilTagSimPublisher ready — {len(self._world_tags)} tags in world, '
            f'range={self._max_range:.1f} m, FOV={math.degrees(self._fov_rad):.0f}°'
        )

    # ------------------------------------------------------------------ #
    #  Timer callback                                                      #
    # ------------------------------------------------------------------ #
    def _publish_cb(self):
        now = self.get_clock().now()

        # Get camera pose in map frame
        try:
            cam_tf = self._tf_buffer.lookup_transform(
                'map', self._camera_frame,
                Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return   # TF not yet available

        cam_x = cam_tf.transform.translation.x
        cam_y = cam_tf.transform.translation.y
        cam_z = cam_tf.transform.translation.z

        # Camera orientation quaternion → yaw
        q = cam_tf.transform.rotation
        cam_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        detections  = AprilTagDetectionArray()
        detections.header.stamp    = now.to_msg()
        detections.header.frame_id = self._camera_frame

        pose_array = PoseArray()
        pose_array.header.stamp    = now.to_msg()
        pose_array.header.frame_id = 'map'

        for tag_data in self._world_tags:
            dx = tag_data['x'] - cam_x
            dy = tag_data['y'] - cam_y
            dz = tag_data['z'] - cam_z

            # Euclidean distance in 3D
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist > self._max_range:
                continue

            # Approximate FOV check: angle between camera forward and tag direction
            tag_angle = math.atan2(dy, dx)
            angle_diff = abs(self._wrap_angle(tag_angle - cam_yaw))
            if angle_diff > self._fov_rad / 2.0:
                continue

            # Transform tag position from map to camera frame
            tag_in_map = PointStamped()
            tag_in_map.header.frame_id = 'map'
            tag_in_map.header.stamp    = now.to_msg()
            tag_in_map.point.x = tag_data['x']
            tag_in_map.point.y = tag_data['y']
            tag_in_map.point.z = tag_data['z']

            try:
                tag_in_cam = self._tf_buffer.transform(
                    tag_in_map, self._camera_frame,
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception:
                continue

            tx = tag_in_cam.point.x
            ty = tag_in_cam.point.y
            tz = tag_in_cam.point.z

            # Build synthetic AprilTagDetection
            # The real detector provides pixel corners + homography.
            # For the sim publisher, we provide centre + four synthetic corners
            # computed from the known tag size.
            tag_info = self._tag_map.get(tag_data['id'], {})
            tag_size = tag_info.get('size', 0.040)  # metres

            # Assume a simple pinhole camera (640×480, 60° FOV → f≈554 px)
            fx = 554.0
            fy = 554.0
            cx = 320.0
            cy = 240.0

            if tz <= 0.01:
                continue

            # Project tag centre to pixel
            u_c = fx * (tx / tz) + cx
            v_c = fy * (ty / tz) + cy

            # Approximate half-size in pixels
            half_px = fx * (tag_size / 2.0) / tz

            det = AprilTagDetection()
            det.family   = '36h11'
            det.id       = tag_data['id']
            det.hamming  = 0
            det.goodness = 1.0
            det.decision_margin = 100.0

            det.centre = TagPoint(x=float(u_c), y=float(v_c))
            det.corners = [
                TagPoint(x=float(u_c - half_px), y=float(v_c - half_px)),  # TL
                TagPoint(x=float(u_c + half_px), y=float(v_c - half_px)),  # TR
                TagPoint(x=float(u_c + half_px), y=float(v_c + half_px)),  # BR
                TagPoint(x=float(u_c - half_px), y=float(v_c + half_px)),  # BL
            ]

            # Build a simple homography: pure scale+translation (flat frontal view)
            s = tag_size * fx / tz
            H = np.array([
                [s,    0.0,  u_c],
                [0.0,  s,    v_c],
                [0.0,  0.0,  1.0],
            ], dtype=np.float64)
            det.homography = H.flatten().tolist()

            detections.detections.append(det)

            # Also add to pose array (for visualisation)
            p = Pose()
            p.position.x = tag_data['x']
            p.position.y = tag_data['y']
            p.position.z = tag_data['z']
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self._det_pub.publish(detections)
        if pose_array.poses:
            self._pose_pub.publish(pose_array)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle >  math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
