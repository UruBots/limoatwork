#!/usr/bin/env python3
import time
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import math

#Nuevos imports
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class LimoAtWorkNode(Node):
    def __init__(self):
        super().__init__('limo_atwork_node')
        # Distancias
        self.filtered_angle = 0.0
        self.prev_error = 0.0

        # Control
        self.kp_ang_align = 0.8
        self.kd_ang_align = 0.4
        self.stop_counter = 0
        self.stop_required = 8   
        self.final_approach_distance = 0.10

        # ---- LATERAL control ----
        self.lateral_target_distance = 0.15  # 5 cm
        self.lateral_ok_counter = 0
        self.lateral_ok_required = 4
        self.max_lateral_speed = 0.08

        # Tolerancias
        self.align_window_deg = 15.0
        self.angle_tol = 0.18   # 10°
        self.align_ok_counter = 0
        self.align_ok_required = 3

        # Estado
        self.docking_phase = "APPROACH"
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        #Nueva instancia
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
        self.arm_pub = self.create_publisher(JointTrajectory, '/goal_joint_trajectory', 10)
        self.docking_active = False
        self.docking_timer = None


        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.laser_scan = None
        
        self.get_logger().info('Limo At Work node started.')
        self.publish_initialpose()

    # TODO: CHANGE COORDINATES
    
    def publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]
        self.initial_pub.publish(msg)
        self.get_logger().info('Initial pose publicada una vez.')

    def send_goal(self, x, y, z, orientation: Quaternion):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info("Esperando al servidor de acciones 'navigate_to_pose'...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("✅ Servidor disponible.")

        self.get_logger().info(f"Enviando meta a x={x:.2f}, y={y:.2f}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('⚠ Meta rechazada por el servidor.')
                return

            self.get_logger().info('✅ Meta aceptada. Esperando resultado...')
            result_future = goal_handle.get_result_async()

            def result_callback(result_future):
                result = result_future.result()
                status = result.status
                if status == 4:  # SUCCEEDED
                    self.get_logger().info('🏁 Meta alcanzada con éxito.')
                    time.sleep(5.0)
                    self.start_docking(distance=0.3)
                elif status == 5:  # CANCELED
                    self.get_logger().warn('⚠ Navegación cancelada.')
                elif status == 6:  # ABORTED
                    self.get_logger().error('❌ Navegación abortada.')
                else:
                    self.get_logger().warn(f'🤔 Estado desconocido: {status}')

            result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def send_goal_backup(self, x, y, z, orientation: Quaternion):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info("Esperando al servidor de acciones 'navigate_to_pose'...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("✅ Servidor disponible.")

        self.get_logger().info(f"Enviando meta a x={x:.2f}, y={y:.2f}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('⚠️ Meta rechazada por el servidor.')
                return

            self.get_logger().info('✅ Meta aceptada. Esperando resultado...')
            result_future = goal_handle.get_result_async()

            def result_callback(result_future):
                result = result_future.result()
                status = result.status
                if status == 4:  # SUCCEEDED
                    self.get_logger().info('🏁 Meta alcanzada con éxito.')
                elif status == 5:  # CANCELED
                    self.get_logger().warn('⚠️ Navegación cancelada.')
                elif status == 6:  # ABORTED
                    self.get_logger().error('❌ Navegación abortada.')
                else:
                    self.get_logger().warn(f'🤔 Estado desconocido: {status}')

            result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def laser_callback(self, data):
        self.laser_scan = data

    def read_april_tags(self, msg):
        # Placeholder for reading April tags logic
        self.get_logger().info('Reading April tags.')
        if not msg.detections:
            return
        tag = msg.detections[0]
        tag_id = tag.id[0]
        self.get_logger().info(f'Detected AprilTag ID: {tag_id}')

    def get_lidar_distance_at_angle(self, target_angle_deg):
        if self.laser_scan is None:
            return None

        target_angle_rad = math.radians(target_angle_deg)

        angle_min = self.laser_scan.angle_min
        angle_inc = self.laser_scan.angle_increment
        ranges = self.laser_scan.ranges
        range_max = self.laser_scan.range_max

        index = int((target_angle_rad - angle_min) / angle_inc)

        if 0 <= index < len(ranges):
            r = ranges[index]
            if 0.01 < r < range_max:
                return r

        return None

    def estimate_wall_angle(self):
        if self.laser_scan is None:
            return None

        ranges = self.laser_scan.ranges
        angle_min = self.laser_scan.angle_min
        angle_inc = self.laser_scan.angle_increment
        range_max = self.laser_scan.range_max

        xs = []
        ys = []

        window_rad = math.radians(self.align_window_deg)

        for i, r in enumerate(ranges):
            if 0.05 < r < range_max:
                theta = angle_min + i * angle_inc

                if abs(theta) < window_rad:
                    x = r * math.cos(theta)
                    y = r * math.sin(theta)
                    xs.append(x)
                    ys.append(y)

        if len(xs) < 10:
            return None

        # Ajuste lineal y = mx + c
        A = np.vstack([xs, np.ones(len(xs))]).T
        m, c = np.linalg.lstsq(A, ys, rcond=None)[0]

        wall_angle = math.atan(m)
        return wall_angle

    def start_docking(self, distance):
        if self.docking_active:
            return

        self.safe_distance = distance
        self.docking_active = True
        self.docking_phase = "APPROACH"   # ← RESET DE FASE

        self.docking_timer = self.create_timer(
            0.05, self.docking_step   # 20 Hz
        )

        self.get_logger().info(
            f"🔗 Docking iniciado (distancia objetivo {distance:.2f} m)"
        )


    def docking_step(self):
        if not self.docking_active or self.laser_scan is None:
            return

        ranges = self.laser_scan.ranges
        angle_min = self.laser_scan.angle_min
        angle_inc = self.laser_scan.angle_increment
        range_max = self.laser_scan.range_max

        center = int((0.0 - angle_min) / angle_inc)
        window = 20

        start = max(center - window, 0)
        end = min(center + window, len(ranges))

        front = [r for r in ranges[start:end] if 0.01 < r < range_max]
        if not front:
            return

        avg_dist = sum(front) / len(front)

        cmd = Twist()

        # =========================
        # FASE 1: APROXIMACIÓN
        # =========================
        if self.docking_phase == "APPROACH":

            error_dist = avg_dist - self.final_approach_distance

            cmd = Twist()

            if avg_dist > self.final_approach_distance:
                # velocidad proporcional suave
                cmd.linear.x = min(0.15, max(0.05, 0.6 * error_dist))
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.stop_counter = 0

            else:
                # ya está cerca → pasar a STOP_NEAR
                self.docking_phase = "STOP_NEAR"
                self.get_logger().info("⏸ Cerca de la caja. Esperando estabilidad.")

            return
        #Secuencia 
        if self.docking_phase == "STOP_NEAR":

            cmd = Twist()
            self.cmd_pub.publish(cmd)  # detener completamente

            # esperar que la distancia sea estable
            if abs(avg_dist - self.final_approach_distance) < 0.015:
                self.stop_counter += 1
            else:
                self.stop_counter = 0

            if self.stop_counter >= self.stop_required:
                self.docking_phase = "ALIGN"
                self.align_ok_counter = 0
                self.get_logger().info("➡️ Comenzando alineamiento fino")

            return

        # =========================
        # FASE 2: ALINEACIÓN
        # =========================
        if self.docking_phase == "ALIGN":
            wall_angle = self.estimate_wall_angle()
            if wall_angle is None:
                return
            # -------- FILTRO LOW PASS --------
            alpha = 0.7
            self.filtered_angle = alpha * self.filtered_angle + (1 - alpha) * wall_angle

            error = self.filtered_angle
            derivative = error - self.prev_error

            angular_cmd = -self.kp_ang_align * error - self.kd_ang_align * derivative
            self.prev_error = error

            # -------- DEAD BAND --------
            if abs(error) < self.angle_tol:
                angular_cmd = 0.0

            # -------- SATURACIÓN --------
            max_ang_speed = 0.2
            angular_cmd = max(min(angular_cmd, max_ang_speed), -max_ang_speed)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = angular_cmd
            self.cmd_pub.publish(cmd)

            # -------- ESTABILIDAD --------
            if abs(error) < self.angle_tol:
                self.align_ok_counter += 1
            else:
                self.align_ok_counter = 0

            if self.align_ok_counter >= self.align_ok_required:
                self.get_logger().info("🛑 Alineación suficiente alcanzada")

                # Detener giro
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

                # Pasar a fase lateral
                self.docking_phase = "LATERAL_APPROACH"
                self.lateral_ok_counter = 0

                return

        # =========================
        # FASE 3: MOVIMIENTO LATERAL
        # =========================
        if self.docking_phase == "LATERAL_APPROACH":

            side_dist = self.get_lidar_distance_at_angle(90)

            if side_dist is None:
                return

            error = side_dist - self.lateral_target_distance

            cmd = Twist()

            if side_dist > self.lateral_target_distance:

                # velocidad proporcional suave
                lateral_speed = max(0.02, min(self.max_lateral_speed, 0.8 * error))

                cmd.linear.y = lateral_speed  # cambiar signo si está invertido
                self.cmd_pub.publish(cmd)

                self.lateral_ok_counter = 0

            else:
                self.lateral_ok_counter += 1

                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)

                if self.lateral_ok_counter >= self.lateral_ok_required:
                    self.get_logger().info("🛑 Aproximación lateral completada")

                    self.stop_docking()
                    self.docking_phase = "DONE"

            return

    def stop_docking(self):
        self.docking_active = False

        if self.docking_timer is not None:
            self.docking_timer.cancel()
            self.docking_timer = None

        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

def decode_mixed_bytes(data: bytes) -> list:
    """Decode raw bytes into a list of tokens (ASCII words and decimal numbers)."""
    tokens = []
    buffer_ascii = []

    def flush_ascii():
        if buffer_ascii:
            tokens.append(''.join(buffer_ascii))
            buffer_ascii.clear()

    for b in data:
        if 32 <= b <= 126:  # printable ASCII
            ch = chr(b)
            # treat parentheses as separate tokens
            if ch in ['(', ')']:
                flush_ascii()
                tokens.append(ch)
            else:
                buffer_ascii.append(ch)
        else:
            flush_ascii()
            tokens.append(str(b))  # non-printable as decimal
    flush_ascii()
    return tokens

def read_bag(bag_path, topic_filter):
    final_objects = []
    final_targets = []
    final_sources = []
    final_destinations = []

    with Reader(bag_path) as reader:
        print('Topics in bag:')
        for conn in reader.connections:
            print(conn.topic, conn.msgtype)

        count = 0
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == topic_filter:
                # ✅ make mutable and modify 5th byte
                mutable_raw = bytearray(rawdata)
                if len(mutable_raw) > 4:
                    mutable_raw[4] = 0x00
                rawdata_modified = bytes(mutable_raw)

                # ✅ now clean
                rawdata_modified = rawdata_modified.replace(b'\x000', b'\x60')
                rawdata_modified = rawdata_modified.replace(b'\x001', b'\x61')
                rawdata_modified = rawdata_modified.replace(b'\x002', b'\x62')
                rawdata_modified = rawdata_modified.replace(b'\x003', b'\x63')
                rawdata_modified = rawdata_modified.replace(b'\x004', b'\x64')
                rawdata_modified = rawdata_modified.replace(b'\x005', b'\x65')
                rawdata_modified = rawdata_modified.replace(b'\x006', b'\x66')
                rawdata_modified = rawdata_modified.replace(b'\x007', b'\x67')
                rawdata_modified = rawdata_modified.replace(b'\x008', b'\x68')
                rawdata_modified = rawdata_modified.replace(b'\x009', b'\x69')
                cleaned_bytes = rawdata_modified.replace(b'\x00', b'')
                tokens = decode_mixed_bytes(cleaned_bytes)
                # print("..................")
                # print(rawdata_modified)
                # print("..................")
                # print(cleaned_bytes)

                objects = []
                targets = []
                sources = []
                destinations = []

                i = 0
                while i < len(tokens) - 4:
                    obj = tokens[i]
                    if obj.isdigit():
                        nxt = tokens[i + 1]

                        # CASE A: next token is '4' -> no explicit target
                        if nxt == '4':
                            src = tokens[i + 2] if i + 2 < len(tokens) else None
                            sep2 = tokens[i + 3] if i + 3 < len(tokens) else None
                            dst = tokens[i + 4] if i + 4 < len(tokens) else None
                            if sep2 == '4':
                                objects.append(obj)
                                targets.append('0')
                                sources.append(src)
                                destinations.append(dst)
                                i += 5
                                continue

                        # CASE B: next token is not '4' -> treat it as explicit target
                        # (can be ')' or '(' or any number like '55', '61', etc.)
                        else:
                            tgt = nxt
                            sep1 = tokens[i + 2] if i + 2 < len(tokens) else None
                            src = tokens[i + 3] if i + 3 < len(tokens) else None
                            sep2 = tokens[i + 4] if i + 4 < len(tokens) else None
                            dst = tokens[i + 5] if i + 5 < len(tokens) else None
                            if sep1 == '4' and sep2 == '4':
                                objects.append(obj)
                                targets.append(tgt)
                                sources.append(src)
                                destinations.append(dst)
                                i += 6
                                continue
                    i += 1

                # Convert Objects and Targets to decimal values
                objects_decimal = []
                targets_decimal = []

                for o in objects:
                    if o.isdigit():
                        objects_decimal.append(int(o))
                    else:
                        # in case something unexpected appears
                        objects_decimal.append(ord(o))

                for t in targets:
                    if t.isdigit():
                        targets_decimal.append(int(t))
                    else:
                        targets_decimal.append(ord(t))

                for i in range (0, len(objects_decimal)):
                    if (objects_decimal[i]) > 94:
                        objects_decimal[i] = objects_decimal[i] - 48

                for i in range (0, len(targets_decimal)):
                    if (targets_decimal[i]) > 94:
                        targets_decimal[i] = targets_decimal[i] - 48

                final_objects = objects_decimal
                final_targets = targets_decimal
                final_sources = sources
                final_destinations = destinations

                count += 1

    return final_objects, final_targets, final_sources, final_destinations

def create_pose(x, y, z, ox, oy, oz, ow):
    """Helper to create a PoseStamped with given coordinates."""
    pose = PoseStamped()
    # Fill header
    pose.header.frame_id = 'map'
    # pose.header.stamp = self.get_clock().now().to_msg()
    # Fill position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # Orientation (no rotation)
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose

pose_dict = {
    "WS01": create_pose(2.15, -0.35, 0.0, 0.0, 0.0, -0.66, 0.75), # orientation solved listassa
    "WS02": create_pose(1.85, 1.20, 0.0, 0.0, 0.0, 0.30, 0.95), # orientation solved listassa 
    "WS03": create_pose(4.0, -0.80, 0.0, 0.0, 0.0, 0.0, 0.99), # orientation solved Listassa
    "WS04": create_pose(5.35, -0.70, 0.0, 0.0, 0.0, -0.99, 0.0), # orientation solved listassa
    "WS05": create_pose(5.15, 2.05, 0.0, 0.0, 0.0, -0.99, 0.0), # orientation solved listassa
    "WS06": create_pose(3.80, 1.85, 0.0, 0.0, 0.0, 0.0, 0.99), # orientation solved listassa
    "WS07": create_pose(2.50, 1.60, 0.0, 0.0, 0.0, -0.91, 0.39),  # orientation solved listassa 
    "WS08": create_pose(2.70, 4.1, 0.0, 0.0, 0.0, 0.68, 0.72), # orientation solved
    "WS09": create_pose(2.80, 4.75, 0.0, 0.0, 0.0, -0.66, 0.75), # orientation solved
    "WS10": create_pose(4.0, 3.75, 0.0, 0.0, 0.0, 0.68, 0.72), # orientation solved
    "WS11": create_pose(4.0, 4.75, 0.0, 0.0, 0.0, -0.66, 0.75), # orientation solved
    "WS12": create_pose(5.4, 8.0, 0.0, 0.0, 0.0, 0.30, 0.95), # orientation solved
    "WS13": create_pose(3.65, 6.3, 0.0, 0.0, 0.0, 0.68, 0.72), # orientation solved
    "WS14": create_pose(0.65, 6.0, 0.0, 0.0, 0.0, -0.99, 0.0), # orientation solved Listassa
    "RT01": create_pose(1.25, 3.5, 0.0, 0.0, 0.0, -0.99, 0.0), # orientation solved
    "SH01": create_pose(0.5, 1.6, 0.0, 0.0, 0.0, -0.66, 0.75), # orientation solved
    "SH02": create_pose(3.0, 6.1, 0.0, 0.0, 0.0, 0.94, 0.32), # orientation solved
    "PP01": create_pose(0.65, 4.65, 0.0, 0.0, 0.0, -0.99, 0.0), # orientation solved
}


def main(args=None):
    rclpy.init(args=args)
    node = LimoAtWorkNode()

    try:
        # 1. Publicar pose inicial
        node.publish_initialpose()
        node.get_logger().info("📍 Pose inicial publicada")

        # 2. Esperar un poco para que AMCL/Nav2 la tome
        time.sleep(4.0)

        # 3. Definir objetivo
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.9994591812584859
        orientation.w = 0.024883810577807426

        x = 1.2450602913615136
        y = 2.205600136766851    
        z = 0.0

        node.get_logger().info(f"🎯 Enviando goal a x={x}, y={y}")
        node.send_goal(x, y, z, orientation)

        # 6. Mantener el nodo activo para recibir feedback/resultados
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido por el usuario")

    finally:
        node.destroy_node()
        rclpy.shutdown()