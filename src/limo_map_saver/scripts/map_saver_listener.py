#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os
from datetime import datetime

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_listener')
        self.subscription = self.create_subscription(
            Empty,
            '/save_map_trigger',  # topic published by the C++ node
            self.listener_callback,
            10)
        self.get_logger().info('"Map Saver" node started. Waiting for trigger on /save_map_trigger...')

    def listener_callback(self, msg):
        self.get_logger().info('Trigger received! Saving map...')

        home_dir = os.path.expanduser('/home/solverbot/mapas')
        map_name = "mapa_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        map_path = os.path.join(home_dir, map_name)  # e.g. /home/solverbot/mapa_...

        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path]

        try:
            subprocess.Popen(command)
            self.get_logger().info(f"map_saver_cli command executed. Saving to {map_path}.yaml/.pgm")
        except Exception as e:
            self.get_logger().error(f"Failed to execute map_saver_cli: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
