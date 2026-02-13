#!/usr/bin/env python3
import math
import yaml
import threading
import time
import statistics
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, BackUp
from action_msgs.msg import GoalStatus


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

        self.declare_parameter("backup_distance", 0.10)
        self.declare_parameter("backup_speed", 0.08)
        self.declare_parameter("backup_timeout_sec", 3.0)

        # -------- Docking interno (LiDAR-only / parede) --------
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("dock_target_distance", 0.05)  # 5 cm da parede
        self.declare_parameter("dock_tol_enter", 0.01)         # entra docked
        self.declare_parameter("dock_tol_exit", 0.02)          # sai docked (histerese)
        self.declare_parameter("dock_ang_tol_m", 0.015)         # |left-right| <= isso

        # cones (em graus)
        self.declare_parameter("dock_front_cone_deg", 3.0)      # +/- 3°
        self.declare_parameter("dock_side_center_deg", 10.0)    # 10°
        self.declare_parameter("dock_side_halfwidth_deg", 1.0)  # +/- 1°

        # ganhos e limites
        self.declare_parameter("dock_kp_lin", 0.4)
        self.declare_parameter("dock_kp_ang", 1.5)
        self.declare_parameter("dock_v_max", 0.015)
        self.declare_parameter("dock_w_max", 0.25)

        # robustez
        self.declare_parameter("dock_min_valid_points", 5)
        self.declare_parameter("dock_control_rate_hz", 20.0)
        self.declare_parameter("dock_timeout_sec", 30.0)

        # quantas vezes tentar dockar por WS
        self.declare_parameter("dock_retries", 2)

        # Load waypoints
        self.route_name = self.get_parameter("route_name").value
        wp_path = self.get_parameter("waypoints_yaml").value

        self.frame_id, self.wp_map, self.route = self._load_waypoints_and_route(wp_path, self.route_name)
        if not self.route:
            raise RuntimeError(f"Rota '{self.route_name}' vazia ou inexistente no YAML.")

        self.get_logger().info(
            f"frame_id='{self.frame_id}', route='{self.route_name}' com {len(self.route)} WS: {self.route}"
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.backup_client = ActionClient(self, BackUp, "/backup")

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
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_scan)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ✅ Start mission in a separate thread
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
                raise RuntimeError(f"WS '{ws_name}' na rota '{route_name}' não existe em waypoints.")

        return frame_id, wp_map, route_list

    # -------------------------
    # Pose helper
    # -------------------------
    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        # stamp 0 evita rejeição por sim_time/desync
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

        self.get_logger().info(f"Publicando Initial Pose: x={x}, y={y}, yaw={yaw}")
        self.initial_pose_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=2.0)

    # -------------------------
    # Action helpers
    # -------------------------
    def _wait_server(self, client: ActionClient, name: str, timeout_sec: float = 10.0) -> bool:
        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f"Action server {name} não disponível.")
            return False
        return True

    def _send_goal_and_wait(self, client: ActionClient, goal_msg, timeout_sec: float) -> bool:
        send_future = client.send_goal_async(goal_msg)
        goal_response_timeout = 30.0
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=goal_response_timeout)

        if not send_future.done():
            self.get_logger().error(f"Timeout ({goal_response_timeout}s) esperando resposta do goal.")
            return False

        gh = send_future.result()
        if gh is None:
            self.get_logger().error("Não recebeu GoalHandle (None).")
            return False

        if not gh.accepted:
            self.get_logger().error("Goal rejeitado (accepted=False).")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error("Timeout esperando resultado. Cancelando...")
            cancel_future = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    # -------------------------
    # Docking interno helpers
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
            # elimina < range_min e também valores absurdos (> range_max), ex: 65.53
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
        Docking fino (parede): alinha com left-right e aproxima até dock_target_distance.
        Retorna True se docked, False se timeout/sem scan.
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
            f"Docking interno iniciado: target={target:.3f}m, scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}"
        )

        docked = False
        t0 = time.time()
        period = 1.0 / max(1.0, rate)

        while rclpy.ok():
            if time.time() - t0 > timeout:
                self._publish_stop()
                self.get_logger().warn("Docking timeout.")
                return False

            # garantir que recebemos scan recente
            scan = self.latest_scan
            if scan is None:
                self._publish_stop()
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # calcula medianas
            front = self._median_in_cone(scan, 0.0, front_cone, min_pts)
            left = self._median_in_cone(scan, +side_center, side_half, min_pts)
            right = self._median_in_cone(scan, -side_center, side_half, min_pts)

            if front is None or left is None or right is None:
                self._publish_stop()
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(period)
                continue

            err_dist = front - target
            err_ang = left - right  # em metros

            # histerese
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

            # controle
            w = clamp(kp_ang * err_ang, -w_max, w_max)

            if abs(err_ang) > ang_tol_m:
                v = 0.0  # alinha primeiro
            else:
                v = clamp(kp_lin * err_dist, 0.0, v_max)  # só avança se err_dist > 0

            # segurança: se passou do ponto, não avança
            if err_dist < -tol_enter:
                v = 0.0

            self._publish_cmd(v, w)

            # spin curto pra callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    # -------------------------
    # Mission loop
    # -------------------------
    def _run_mission(self):
        self.get_logger().info("Aguardando estabilização (2s) antes do primeiro goal...")
        rclpy.spin_once(self, timeout_sec=2.0)

        self._publish_initial_pose()

        dock_retries = int(self.get_parameter("dock_retries").value)
        nav_timeout = float(self.get_parameter("nav_timeout_sec").value)

        backup_distance = float(self.get_parameter("backup_distance").value)
        backup_speed = float(self.get_parameter("backup_speed").value)
        backup_timeout = float(self.get_parameter("backup_timeout_sec").value)

        if not self._wait_server(self.nav_client, "/navigate_to_pose"):
            return
        if not self._wait_server(self.backup_client, "/backup"):
            return

        for i, ws_name in enumerate(self.route, start=1):
            wp = self.wp_map[ws_name]
            self.get_logger().info(
                f"[{i}/{len(self.route)}] Indo para {wp.name} (x={wp.x:.3f}, y={wp.y:.3f}, yaw={wp.yaw:.3f})"
            )

            # 1) NavigateToPose (macro)
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = self._pose_stamped(wp.x, wp.y, wp.yaw)

            if not self._send_goal_and_wait(self.nav_client, nav_goal, timeout_sec=nav_timeout):
                self.get_logger().error(f"{wp.name}: navegação falhou. Pulando.")
                continue

            # 2) Docking interno (fino)
            dock_ok = False
            for attempt in range(1, dock_retries + 1):
                self.get_logger().info(f"{wp.name}: docking interno tentativa {attempt}/{dock_retries}")
                if self._dock_to_wall():
                    dock_ok = True
                    break

            if not dock_ok:
                self.get_logger().error(f"{wp.name}: docking interno falhou. Indo para próxima WS.")
                continue

            # 3) BackUp (undock)
            self.get_logger().info(f"{wp.name}: undock (backup {backup_distance:.2f} m)")
            b = BackUp.Goal()
            b.target = Point(x=abs(backup_distance), y=0.0, z=0.0)
            b.speed = float(backup_speed)
            b.time_allowance.sec = int(backup_timeout)
            b.time_allowance.nanosec = 0

            if not self._send_goal_and_wait(self.backup_client, b, timeout_sec=backup_timeout + 5.0):
                self.get_logger().warn(f"{wp.name}: backup falhou (seguindo mesmo assim).")

            self.get_logger().info(f"{wp.name}: OK ✅")

        self.get_logger().info("Rota finalizada ✅")


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
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
