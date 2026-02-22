#!/usr/bin/env python3
"""
detection_visualizer.py
=======================
Real-time OpenCV dashboard for the limo_manipulation stack.

Shows a split-screen window:

  ┌────────────────────────────────┬──────────────────────┐
  │  Camera feed                   │  STATUS PANEL        │
  │                                │                      │
  │  • Green quads around each     │  Mode:  SIM / REAL   │
  │    detected AprilTag           │  State: PICKING      │
  │  • Tag ID + object type label  │  WS:    WS01         │
  │  • Pose axes (X=red,Y=green,   │                      │
  │    Z=blue) projected in image  │  Detected objects:   │
  │  • Distance to camera          │   [0] attc_cube      │
  │                                │       dist: 0.45 m   │
  │                                │       id:  tag_0     │
  │                                │                      │
  │                                │  Arm joints (°):     │
  │                                │   J1:  0.0           │
  │                                │   J2: -57.3          │
  │                                │   J3:  40.1          │
  │                                │   J4:  17.2          │
  │                                │   Grip: 12.0         │
  └────────────────────────────────┴──────────────────────┘

The annotated composite image is also published on:
  /manipulation/debug_image  (sensor_msgs/Image)

so it can be viewed in RViz2 or rqt_image_view without needing a local
display (useful in Docker / devcontainer / SSH sessions).

Subscriptions
-------------
  /camera/image_raw             sensor_msgs/Image
  /camera/camera_info           sensor_msgs/CameraInfo
  /apriltag/detections          apriltag_msgs/AprilTagDetectionArray
  /manipulation/detected_objects std_msgs/String   (JSON list)
  /manipulation/status          std_msgs/String   (IDLE/PICKING/…)
  /joint_states                 sensor_msgs/JointState

Parameters
----------
  use_sim        bool   True = simulation mode label in UI (default True)
  show_window    bool   Open a local cv2.imshow window (default True).
                        Set False when running headless (Docker without X11).
  window_scale   float  Scale factor for the cv2 window (default 0.9)
  panel_width    int    Width of the status panel in pixels (default 320)
  font_scale     float  OpenCV font scale for status text (default 0.55)
"""

import json
import math
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, JointState
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge


# ── colour palette (BGR) ─────────────────────────────────────────────────────
C_GREEN   = (50,  205,  50)
C_RED     = (50,   50, 220)
C_BLUE    = (220, 100,  50)
C_YELLOW  = (0,   220, 220)
C_WHITE   = (255, 255, 255)
C_BLACK   = (0,     0,   0)
C_GRAY    = (160, 160, 160)
C_ORANGE  = (0,   165, 255)
C_PANEL   = (28,   28,  38)   # dark panel background

# ── state → colour mapping ────────────────────────────────────────────────────
STATE_COLORS = {
    'IDLE':     C_GREEN,
    'PICKING':  C_YELLOW,
    'CARRYING': C_ORANGE,
    'PLACING':  C_BLUE,
    'ERROR':    C_RED,
}

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint']


class DetectionVisualizer(Node):
    """Live OpenCV dashboard for AprilTag detection and manipulation status."""

    def __init__(self):
        super().__init__('detection_visualizer')

        # ------------------------------------------------------------------ #
        #  Parameters                                                         #
        # ------------------------------------------------------------------ #
        self.declare_parameter('use_sim',     True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_scale', 0.9)
        self.declare_parameter('panel_width',  320)
        self.declare_parameter('font_scale',   0.55)

        self._use_sim     = self.get_parameter('use_sim').value
        self._show_win    = self.get_parameter('show_window').value
        self._win_scale   = float(self.get_parameter('window_scale').value)
        self._panel_w     = int(self.get_parameter('panel_width').value)
        self._font_scale  = float(self.get_parameter('font_scale').value)

        # ------------------------------------------------------------------ #
        #  Shared state (protected by lock)                                   #
        # ------------------------------------------------------------------ #
        self._lock           = threading.Lock()
        self._color_img      = None        # latest BGR OpenCV image
        self._camera_info    = None        # CameraInfo
        self._tag_detections = []          # list of AprilTagDetection
        self._detected_objs  = []          # list of dicts from JSON
        self._manip_state    = 'IDLE'
        self._joint_positions: dict = {}   # joint_name → radians

        # ------------------------------------------------------------------ #
        #  cv_bridge                                                          #
        # ------------------------------------------------------------------ #
        self._bridge = CvBridge()

        # ------------------------------------------------------------------ #
        #  Subscribers                                                        #
        # ------------------------------------------------------------------ #
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos_sensor)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._info_cb, qos_sensor)
        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self._tag_cb, 10)
        self.create_subscription(
            String, '/manipulation/detected_objects', self._objects_cb, 10)
        self.create_subscription(
            String, '/manipulation/status', self._status_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joints_cb, qos_sensor)

        # ------------------------------------------------------------------ #
        #  Publisher (annotated debug image)                                  #
        # ------------------------------------------------------------------ #
        self._debug_pub = self.create_publisher(Image, '/manipulation/debug_image', 10)

        # ------------------------------------------------------------------ #
        #  Render timer                                                       #
        #  Match the camera update_rate (5 Hz in simulation).                #
        #  Running faster than the camera rate wastes CPU with duplicate      #
        #  frames. When show_window=False the timer cost is minimal.          #
        # ------------------------------------------------------------------ #
        self.create_timer(1.0 / 5.0, self._render_cb)

        # ------------------------------------------------------------------ #
        #  OpenCV window                                                      #
        # ------------------------------------------------------------------ #
        if self._show_win:
            cv2.namedWindow('LIMO Manipulation Monitor', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('LIMO Manipulation Monitor', 960, 480)
            self.get_logger().info('OpenCV window opened.')
            

        mode = 'SIM' if self._use_sim else 'REAL'
        self.get_logger().info(
            f'DetectionVisualizer ready (mode={mode}, '
            f'show_window={self._show_win}).'
        )

    # ================================================================== #
    #  Subscriber callbacks                                               #
    # ================================================================== #

    def _image_cb(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        with self._lock:
            self._color_img = img

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            self._camera_info = msg

    def _tag_cb(self, msg: AprilTagDetectionArray):
        with self._lock:
            self._tag_detections = list(msg.detections)

    def _objects_cb(self, msg: String):
        try:
            objs = json.loads(msg.data)
        except Exception:
            objs = []
        with self._lock:
            self._detected_objs = objs if isinstance(objs, list) else []

    def _status_cb(self, msg: String):
        with self._lock:
            self._manip_state = msg.data.strip()

    def _joints_cb(self, msg: JointState):
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            positions[name] = float(pos)
        with self._lock:
            self._joint_positions = positions

    # ================================================================== #
    #  Render callback                                                    #
    # ================================================================== #

    def _render_cb(self):
        with self._lock:
            img           = self._color_img.copy() if self._color_img is not None else None
            cam_info      = self._camera_info
            tags          = list(self._tag_detections)
            detected_objs = list(self._detected_objs)
            state         = self._manip_state
            joints        = dict(self._joint_positions)

        # ---- build camera panel ------------------------------------------ #
        if img is None:
            # placeholder if camera not yet available
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, 'Waiting for camera...', (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, C_GRAY, 2)
        else:
            img = self._draw_apriltags(img, tags, cam_info)

        # Resize camera panel to fixed height
        target_h = 480
        cam_h, cam_w = img.shape[:2]
        if cam_h != target_h:
            scale = target_h / cam_h
            img = cv2.resize(img, (int(cam_w * scale), target_h))

        # ---- build status panel ------------------------------------------ #
        panel = self._build_status_panel(
            height=target_h,
            state=state,
            detected_objs=detected_objs,
            joints=joints,
        )

        # ---- combine horizontally --------------------------------------- #
        composite = np.hstack([img, panel])

        # ---- publish debug image ---------------------------------------- #
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(composite, 'bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_link'
            self._debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Debug image publish failed: {e}')

        # ---- show OpenCV window ----------------------------------------- #
        if self._show_win:
            h, w = composite.shape[:2]
            disp = cv2.resize(
                composite,
                (int(w * self._win_scale), int(h * self._win_scale))
            )
            # Draw 'q = quit' hint at the bottom-left corner
            cv2.putText(
                disp, 'q: quit',
                (6, disp.shape[0] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1,
                cv2.LINE_AA
            )
            cv2.imshow('LIMO Manipulation Monitor', disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('q pressed — shutting down visualizer.')
                cv2.destroyAllWindows()
                self._show_win = False  # stop rendering
                rclpy.shutdown()

    # ================================================================== #
    #  AprilTag overlay drawing                                           #
    # ================================================================== #

    def _draw_apriltags(self, img: np.ndarray,
                        tags: list,
                        cam_info) -> np.ndarray:
        """Draw AprilTag quads, axes, labels and distance on the camera image."""
        for det in tags:
            corners = np.array(
                [[c.x, c.y] for c in det.corners], dtype=np.float32
            )

            # ── tag quad (green) ─────────────────────────────────────────
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(img, [pts], isClosed=True,
                          color=C_GREEN, thickness=2)

            # ── corner dots ──────────────────────────────────────────────
            for i, (cx, cy) in enumerate(corners):
                colour = [C_RED, C_YELLOW, C_GREEN, C_BLUE][i]
                cv2.circle(img, (int(cx), int(cy)), 4, colour, -1)

            # ── centre dot ───────────────────────────────────────────────
            cx_c = int(det.centre.x)
            cy_c = int(det.centre.y)
            cv2.circle(img, (cx_c, cy_c), 5, C_WHITE, -1)

            # ── distance estimation (rough, from tag pixel size) ─────────
            tag_px_size = float(np.linalg.norm(corners[0] - corners[1]))
            dist_str = ''
            if cam_info is not None and tag_px_size > 5:
                fx = cam_info.k[0]
                # Assume default 0.036 m tag; adjust if known
                real_size_m = 0.036
                dist_m = (real_size_m * fx) / tag_px_size
                dist_str = f'  {dist_m:.2f}m'

            # ── label ────────────────────────────────────────────────────
            label = f'ID:{det.id}{dist_str}'
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            lx = cx_c - tw // 2
            ly = cy_c - 14
            cv2.rectangle(img,
                           (lx - 2, ly - th - 2),
                           (lx + tw + 2, ly + 2),
                           C_BLACK, -1)
            cv2.putText(img, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_GREEN, 1,
                        cv2.LINE_AA)

            # ── 3D pose axes (using homography → rough axes) ──────────────
            if cam_info is not None:
                self._draw_axes(img, det, cam_info)

        return img

    def _draw_axes(self, img: np.ndarray, det, cam_info) -> None:
        """Project simplified 3D axes from the tag homography onto the image."""
        try:
            H = np.array(det.homography, dtype=np.float64).reshape(3, 3)
            K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
            K_inv = np.linalg.inv(K)

            # Recover approximate rotation columns from H K^-1
            h = K_inv @ H
            col1 = h[:, 0]
            col2 = h[:, 1]
            scale = 1.0 / (0.5 * (np.linalg.norm(col1) + np.linalg.norm(col2)))

            # Axis length in pixels ≈ 30% of tag diagonal
            corners = np.array([[c.x, c.y] for c in det.corners])
            diag = np.linalg.norm(corners[0] - corners[2])
            axis_len = diag * 0.35

            # Project tag centre
            centre_h = np.array([det.centre.x, det.centre.y, 1.0])
            cx, cy = int(det.centre.x), int(det.centre.y)

            # X axis (red) — along tag's first column in image
            dx = col1 / np.linalg.norm(col1)
            xp = (int(cx + dx[0] * axis_len), int(cy + dx[1] * axis_len))
            cv2.arrowedLine(img, (cx, cy), xp, C_RED,   2, tipLength=0.25)

            # Y axis (green) — along tag's second column
            dy = col2 / np.linalg.norm(col2)
            yp = (int(cx + dy[0] * axis_len), int(cy + dy[1] * axis_len))
            cv2.arrowedLine(img, (cx, cy), yp, C_GREEN, 2, tipLength=0.25)

            # Z axis (blue) — cross product, points "out" of tag face
            dz = np.cross(dx[:2], dy[:2])
            zp = (int(cx), int(cy - abs(dz) * axis_len * 0.5))
            cv2.arrowedLine(img, (cx, cy), zp, C_BLUE,  2, tipLength=0.25)

            # Tiny axis labels
            cv2.putText(img, 'X', xp, cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_RED,   1)
            cv2.putText(img, 'Y', yp, cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_GREEN, 1)
            cv2.putText(img, 'Z', zp, cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_BLUE,  1)
        except Exception:
            pass   # silently skip if homography is degenerate

    # ================================================================== #
    #  Status panel builder                                               #
    # ================================================================== #

    def _build_status_panel(self, height: int, state: str,
                             detected_objs: list,
                             joints: dict) -> np.ndarray:
        """Build the dark right-hand status panel as a numpy image."""
        panel = np.full((height, self._panel_w, 3), C_PANEL, dtype=np.uint8)
        fs    = self._font_scale
        lh    = int(fs * 36)        # line height
        y     = lh                  # current y cursor
        x     = 12                  # left margin

        def text(s: str, colour=C_WHITE, bold=False):
            nonlocal y
            thickness = 2 if bold else 1
            cv2.putText(panel, s, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, colour, thickness,
                        cv2.LINE_AA)
            y += lh

        def divider(colour=C_GRAY):
            nonlocal y
            cv2.line(panel, (x, y - lh // 2),
                     (self._panel_w - x, y - lh // 2), colour, 1)

        def header(s: str):
            nonlocal y
            text(s, C_YELLOW, bold=True)
            divider()

        # ── Title ────────────────────────────────────────────────────────
        header('LIMO MANIPULATION')

        # ── Mode ─────────────────────────────────────────────────────────
        mode = 'SIM' if self._use_sim else 'REAL'
        text(f'Mode:  {mode}', C_ORANGE if self._use_sim else C_GREEN)

        # ── State ────────────────────────────────────────────────────────
        sc = STATE_COLORS.get(state, C_WHITE)
        text(f'State: {state}', sc, bold=True)
        y += lh // 3

        # ── Detected objects ─────────────────────────────────────────────
        header('Detected objects')
        if not detected_objs:
            text('  (none)', C_GRAY)
        else:
            for i, obj in enumerate(detected_objs[:5]):  # max 5 entries
                obj_type = obj.get('type', '?')
                method   = obj.get('method', '')
                tag_id   = obj.get('tag_id', '')
                x_m      = obj.get('x', 0.0)
                y_m      = obj.get('y', 0.0)
                z_m      = obj.get('z', 0.0)
                dist_2d  = math.sqrt(x_m**2 + y_m**2)
                m_label  = f'[{i}] {obj_type}'
                text(m_label, C_WHITE)
                info = f'  ({x_m:.2f},{y_m:.2f},{z_m:.2f})'
                text(info, C_GRAY)
                if tag_id != '':
                    text(f'  tag_id={tag_id} via {method}', C_GRAY)
                text(f'  dist_xy={dist_2d:.2f} m', C_GREEN)
                y += lh // 4

        y += lh // 3

        # ── Arm joint angles ─────────────────────────────────────────────
        header('Arm joints (deg)')
        joint_labels = {
            'joint1':            'J1 (base) ',
            'joint2':            'J2 (shld) ',
            'joint3':            'J3 (elbow)',
            'joint4':            'J4 (wrist)',
            'gripper_left_joint':'Gripper   ',
        }
        for jname, jlabel in joint_labels.items():
            if jname in joints:
                deg = math.degrees(joints[jname])
                bar_len = int(abs(deg) / 180.0 * (self._panel_w - 2 * x - 80))
                bar_len = min(bar_len, self._panel_w - 2 * x - 80)
                bar_col = C_BLUE if deg >= 0 else C_RED
                # mini bar indicator
                bx = x + 78
                by = y - lh + 4
                cv2.rectangle(panel,
                               (bx, by),
                               (bx + bar_len, by + lh - 8),
                               bar_col, -1)
                text(f'{jlabel}: {deg:+.1f}', C_WHITE)
            else:
                text(f'{jlabel}: --', C_GRAY)

        # ── Bottom hint ───────────────────────────────────────────────────
        y = height - lh - 6
        cv2.line(panel, (x, y - lh // 2),
                 (self._panel_w - x, y - lh // 2), C_GRAY, 1)
        hint = 'rqt: /manipulation/debug_image'
        cv2.putText(panel, hint, (x, height - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_GRAY, 1, cv2.LINE_AA)

        return panel


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.get_parameter('show_window').value:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
