#!/usr/bin/env python3
"""
object_detector.py
==================
ROS 2 node that detects @Work objects and reports their 3D poses.

Detection pipeline summary
--------------------------

  ┌──────────────┐    ┌─────────────────────────┐    ┌──────────────────┐
  │  Image source│    │  AprilTag detector       │    │  Pose estimator  │
  │              │    │                          │    │                  │
  │ Sim: Gazebo  │──▶ │  use_sim=true:           │──▶ │  OpenCV PnP      │
  │   RGB camera │    │    apriltag_sim_publisher│    │  (solvePnP with  │
  │              │    │                          │    │   known tag size)│
  │ Real: Rense  │──▶ │  use_sim=false:          │    │                  │
  │   D435/D455  │    │    apriltag_detector node│    │  → 3D pose in    │
  └──────────────┘    └─────────────────────────┘    │    camera frame  │
                                                      │  → TF to 'map'  │
                                                      └──────────────────┘
  Fallback (use_sim=true, no camera image):
    ● Returns known positions from objects_config.yaml (sim_object_positions)

Detection modes
---------------
  1. AprilTag (primary, both sim and real):
     - Subscribes to /apriltag/detections (AprilTagDetectionArray)
     - Subscribes to /camera/camera_info for intrinsics
     - Estimates 3D pose per tag via solvePnP (known physical tag size)
     - Transforms to 'map' frame via TF2

  2. HSV colour (real robot fallback, use_sim=false only):
     - Runs when no AprilTag is detected within detect timeout
     - Uses depth channel for 3D position
     - Lower precision than AprilTag mode

  3. Config-based mock (simulation fallback):
     - Used when use_sim=true AND no tag detected within timeout
     - Returns positions from objects_config.yaml → sim_object_positions

Provides service
----------------
  /manipulation/detect_objects  (std_srvs/Trigger)
    Returns success=True if at least one object found.
    response.message = JSON list of detections.

Publishes
---------
  /manipulation/detected_objects  (std_msgs/String)  JSON detections
  /manipulation/detection_debug   (sensor_msgs/Image) annotated image
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo

from apriltag_msgs.msg import AprilTagDetectionArray

import yaml

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401
    from geometry_msgs.msg import PointStamped, PoseStamped
    _cv_available = True
except ImportError:
    _cv_available = False


class ObjectDetector(Node):
    """Detects @Work objects using AprilTags and/or HSV colour segmentation."""

    def __init__(self):
        super().__init__('object_detector')

        # ------------------------------------------------------------------ #
        #  Parameters                                                         #
        # ------------------------------------------------------------------ #
        self.declare_parameter('use_sim',         True)
        self.declare_parameter('workspace',       'WS01')
        self.declare_parameter('objects_config',  '')
        self.declare_parameter('apriltag_config', '')
        self.declare_parameter('camera_frame',    'camera_color_optical_frame')
        self.declare_parameter('target_frame',    'map')
        self.declare_parameter('depth_scale',     0.001)
        self.declare_parameter('tag_timeout_sec', 3.0)   # seconds to wait for tag

        self._use_sim      = self.get_parameter('use_sim').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._target_frame = self.get_parameter('target_frame').value

        # ------------------------------------------------------------------ #
        #  Load YAML configs                                                  #
        # ------------------------------------------------------------------ #
        self._obj_cfg: dict = {}
        self._ap_cfg:  dict = {}
        self._tag_map: dict = {}   # tag_id → tag config dict

        obj_path = self.get_parameter('objects_config').value
        if obj_path and os.path.isfile(obj_path):
            with open(obj_path, 'r') as f:
                self._obj_cfg = yaml.safe_load(f)
            self.get_logger().info(f'Loaded objects config: {obj_path}')
        else:
            self.get_logger().warn(f'objects_config not found: "{obj_path}"')

        ap_path = self.get_parameter('apriltag_config').value
        if ap_path and os.path.isfile(ap_path):
            with open(ap_path, 'r') as f:
                self._ap_cfg = yaml.safe_load(f)
            for tag in self._ap_cfg.get('tags', []):
                self._tag_map[tag['id']] = tag
            self.get_logger().info(
                f'Loaded {len(self._tag_map)} AprilTag definitions: {ap_path}')
        else:
            self.get_logger().warn(f'apriltag_config not found: "{ap_path}"')

        # ------------------------------------------------------------------ #
        #  Shared state (thread-safe via lock)                                #
        # ------------------------------------------------------------------ #
        self._lock           = threading.Lock()
        self._camera_info    = None    # sensor_msgs/CameraInfo
        self._color_img      = None    # sensor_msgs/Image
        self._depth_img      = None    # sensor_msgs/Image
        self._latest_tags    = None    # AprilTagDetectionArray
        self._last_tag_stamp = None    # rclpy.time.Time of latest tag message

        # ------------------------------------------------------------------ #
        #  TF2                                                                #
        # ------------------------------------------------------------------ #
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------ #
        #  CV bridge (real-robot HSV fallback)                                #
        # ------------------------------------------------------------------ #
        if _cv_available:
            self._bridge = CvBridge()
        else:
            self.get_logger().warn(
                'cv_bridge / OpenCV not available. HSV fallback disabled.')

        # ------------------------------------------------------------------ #
        #  Subscribers                                                        #
        # ------------------------------------------------------------------ #
        # AprilTag detections (from apriltag_detector OR apriltag_sim_publisher)
        self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self._tag_cb, 10)

        # Camera info (needed for PnP pose estimation)
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._info_cb, 10)

        if not self._use_sim and _cv_available:
            # Real robot: also subscribe to colour + depth for HSV fallback
            self.create_subscription(
                Image, '/camera/color/image_raw',    self._color_cb, 10)
            self.create_subscription(
                Image, '/camera/depth/image_rect_raw', self._depth_cb, 10)

        # ------------------------------------------------------------------ #
        #  Publishers                                                         #
        # ------------------------------------------------------------------ #
        self._det_pub   = self.create_publisher(String, '/manipulation/detected_objects', 10)
        self._debug_pub = self.create_publisher(Image,  '/manipulation/detection_debug',  10)

        # ------------------------------------------------------------------ #
        #  Service                                                            #
        # ------------------------------------------------------------------ #
        self.create_service(Trigger, '/manipulation/detect_objects', self._detect_cb)

        self.get_logger().info(
            f'ObjectDetector ready '
            f'({"sim" if self._use_sim else "real"} mode, '
            f'{len(self._tag_map)} known tags).'
        )

    # ================================================================== #
    #  Subscriber callbacks                                               #
    # ================================================================== #
    def _tag_cb(self, msg: AprilTagDetectionArray):
        with self._lock:
            self._latest_tags    = msg
            self._last_tag_stamp = self.get_clock().now()

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            self._camera_info = msg

    def _color_cb(self, msg):
        with self._lock:
            self._color_img = msg

    def _depth_cb(self, msg):
        with self._lock:
            self._depth_img = msg

    # ================================================================== #
    #  Service handler                                                    #
    # ================================================================== #
    def _detect_cb(self, request, response):
        workspace   = self.get_parameter('workspace').value
        tag_timeout = self.get_parameter('tag_timeout_sec').value

        # 1. Try AprilTag detection (primary method)
        detections = self._detect_from_apriltags(tag_timeout)

        # 2. Fallback: HSV colour (real robot) or config mock (simulation)
        if not detections:
            if self._use_sim:
                self.get_logger().info(
                    '[detect] No AprilTags seen — using config-based mock positions.')
                detections = self._detect_from_config(workspace)
            elif _cv_available:
                self.get_logger().info(
                    '[detect] No AprilTags seen — trying HSV colour fallback.')
                detections = self._detect_from_hsv(workspace)

        payload = json.dumps(detections)
        msg = String()
        msg.data = payload
        self._det_pub.publish(msg)

        response.success = len(detections) > 0
        response.message = payload
        self.get_logger().info(
            f'[{workspace}] {len(detections)} object(s) detected '
            f'({"tags" if detections else "none"}).'
        )
        return response

    # ================================================================== #
    #  Detection method 1: AprilTag → PnP pose                           #
    # ================================================================== #
    def _detect_from_apriltags(self, timeout_sec: float) -> list:
        """
        Wait up to timeout_sec for fresh AprilTag detections, then estimate
        3D pose for each visible tag using solvePnP (known physical tag size).
        """
        if not _cv_available:
            return []

        # Wait for a fresh tag message
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                tags      = self._latest_tags
                tag_stamp = self._last_tag_stamp
                cam_info  = self._camera_info
            if tags is not None and len(tags.detections) > 0:
                break
            time.sleep(0.1)

        with self._lock:
            tags     = self._latest_tags
            cam_info = self._camera_info

        if tags is None or len(tags.detections) == 0:
            return []
        if cam_info is None:
            self.get_logger().warn('[apriltag] No camera_info available for PnP.')
            return []

        # Camera intrinsic matrix
        K  = np.array(cam_info.k).reshape(3, 3)
        fx = K[0, 0]; fy = K[1, 1]
        cx = K[0, 2]; cy = K[1, 2]
        dist_coeffs = np.array(cam_info.d, dtype=np.float64)

        # AprilTag pose estimation settings
        min_margin = self._ap_cfg.get('pose', {}).get('min_decision_margin', 20.0)
        max_dist   = self._ap_cfg.get('pose', {}).get('max_detection_distance', 1.2)

        detections = []
        for det in tags.detections:
            tag_info = self._tag_map.get(det.id)
            if tag_info is None:
                continue  # unknown tag ID
            if det.decision_margin < min_margin:
                continue  # low confidence

            tag_size = float(tag_info.get('size', 0.040))  # physical size in metres
            half     = tag_size / 2.0

            # 3D object points of tag corners (tag plane, Z=0)
            obj_pts = np.array([
                [-half,  half, 0.0],  # TL
                [ half,  half, 0.0],  # TR
                [ half, -half, 0.0],  # BR
                [-half, -half, 0.0],  # BL
            ], dtype=np.float64)

            # 2D image points from detection corners
            img_pts = np.array([
                [det.corners[0].x, det.corners[0].y],
                [det.corners[1].x, det.corners[1].y],
                [det.corners[2].x, det.corners[2].y],
                [det.corners[3].x, det.corners[3].y],
            ], dtype=np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, K, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            # tvec is tag centre in camera optical frame (metres)
            tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
            dist_3d = np.sqrt(tx*tx + ty*ty + tz*tz)
            if dist_3d > max_dist:
                continue   # too far — unreliable

            # Apply tag→object offset to find the grasp centre
            offset = tag_info.get('tag_to_object_offset', [0.0, 0.0, 0.0])
            # offset is in tag local frame (z = normal direction outward from tag face)
            # In camera frame we need to rotate offset by tag orientation
            R_mat, _ = cv2.Rodrigues(rvec)
            off_world = R_mat @ np.array(offset)
            grasp_in_cam = np.array([tx, ty, tz]) + off_world

            # Transform grasp point to target_frame (map)
            pt_cam = PointStamped()
            pt_cam.header.frame_id = self._camera_frame
            pt_cam.header.stamp    = self.get_clock().now().to_msg()
            pt_cam.point.x = float(grasp_in_cam[0])
            pt_cam.point.y = float(grasp_in_cam[1])
            pt_cam.point.z = float(grasp_in_cam[2])

            try:
                pt_map = self._tf_buffer.transform(
                    pt_cam, self._target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.3))
            except Exception as e:
                self.get_logger().warn(f'TF transform failed for tag {det.id}: {e}')
                continue

            # Rotation: convert rvec to quaternion for orientation reporting
            theta = float(np.linalg.norm(rvec))
            if theta > 1e-6:
                axis = rvec.flatten() / theta
                qx = float(axis[0] * np.sin(theta/2))
                qy = float(axis[1] * np.sin(theta/2))
                qz = float(axis[2] * np.sin(theta/2))
                qw = float(np.cos(theta/2))
            else:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

            detections.append({
                'id':         f'tag_{det.id}_{tag_info["object_type"]}',
                'type':       tag_info['object_type'],
                'tag_id':     det.id,
                'x':          float(pt_map.point.x),
                'y':          float(pt_map.point.y),
                'z':          float(pt_map.point.z),
                'qx':         qx,  'qy': qy, 'qz': qz, 'qw': qw,
                'yaw':        0.0,  # approximate; use quat for full orientation
                'frame_id':   self._target_frame,
                'confidence': float(det.decision_margin),
                'method':     'apriltag',
            })
            self.get_logger().info(
                f'  Tag {det.id} ({tag_info["object_type"]}): '
                f'({pt_map.point.x:.3f}, {pt_map.point.y:.3f}, {pt_map.point.z:.3f}) '
                f'dist_cam={dist_3d:.3f} m  margin={det.decision_margin:.1f}'
            )

        return detections

    # ================================================================== #
    #  Detection method 2: Config mock (simulation fallback)              #
    # ================================================================== #
    def _detect_from_config(self, workspace: str) -> list:
        """Return known positions from sim_object_positions in objects_config."""
        sim_positions = self._obj_cfg.get('sim_object_positions', {})
        result = []
        for name, data in sim_positions.items():
            if data.get('workspace') == workspace:
                pos = data['position']
                result.append({
                    'id':       name,
                    'type':     data.get('type', 'unknown'),
                    'x':        pos[0],
                    'y':        pos[1],
                    'z':        pos[2],
                    'yaw':      data.get('yaw', 0.0),
                    'frame_id': 'map',
                    'method':   'config_mock',
                })
        return result

    # ================================================================== #
    #  Detection method 3: HSV colour + depth (real robot fallback)       #
    # ================================================================== #
    def _detect_from_hsv(self, workspace: str) -> list:
        """HSV colour segmentation on RealSense colour + depth images."""
        if not _cv_available:
            return []

        with self._lock:
            color_msg  = self._color_img
            depth_msg  = self._depth_img
            info_msg   = self._camera_info

        if color_msg is None or depth_msg is None or info_msg is None:
            self.get_logger().warn('[HSV] No camera images available yet.')
            return []

        color_cv = self._bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_cv = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        fx = info_msg.k[0]; fy = info_msg.k[4]
        cx = info_msg.k[2]; cy = info_msg.k[5]
        depth_scale = self.get_parameter('depth_scale').value

        hsv_img     = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        objects_cfg = self._obj_cfg.get('objects', {})
        detections  = []

        for obj_type, obj_cfg in objects_cfg.items():
            low  = np.array(obj_cfg['color_hsv_low'],  dtype=np.uint8)
            high = np.array(obj_cfg['color_hsv_high'], dtype=np.uint8)
            mask = cv2.inRange(hsv_img, low, high)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 200:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                d = float(depth_cv[v, u]) * depth_scale
                if d <= 0.01 or d > 1.5:
                    continue

                x_cam = (u - cx) * d / fx
                y_cam = (v - cy) * d / fy
                z_cam = d

                pt_camera = PointStamped()
                pt_camera.header.frame_id = self._camera_frame
                pt_camera.header.stamp    = self.get_clock().now().to_msg()
                pt_camera.point.x = x_cam
                pt_camera.point.y = y_cam
                pt_camera.point.z = z_cam

                try:
                    pt_target = self._tf_buffer.transform(
                        pt_camera, self._target_frame,
                        timeout=rclpy.duration.Duration(seconds=0.3))
                    detections.append({
                        'id':       f'{obj_type}_hsv',
                        'type':     obj_type,
                        'x':        float(pt_target.point.x),
                        'y':        float(pt_target.point.y),
                        'z':        float(pt_target.point.z),
                        'yaw':      0.0,
                        'frame_id': self._target_frame,
                        'method':   'hsv',
                    })
                except Exception as e:
                    self.get_logger().warn(f'HSV TF transform failed: {e}')

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
