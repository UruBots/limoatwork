#!/usr/bin/env python3
"""
object_spawner.py -- Spawn manipulation objects in Gazebo Sim.

Reads a preset from atwork_arena_description/config/object_spawns.yaml
and spawns each object on the corresponding table at the correct height
using `ros2 run ros_gz_sim create`.

Usage (standalone):
  ros2 run limo_mission object_spawner --ros-args -p preset:=BTT1

Usage (launch integration):
  Node(package='limo_mission', executable='object_spawner',
       parameters=[{'preset': 'BTT1', 'world_name': 'atwork_2025_world'}])

Presets available: BMT, BTT1, BTT2, ATT1, ATT2, FINAL
"""

import os
import subprocess
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class ObjectSpawner(Node):

    def __init__(self):
        super().__init__('object_spawner')

        self.declare_parameter('preset', 'BTT1')
        self.declare_parameter('spawns_yaml', '')
        self.declare_parameter('world_name', 'atwork_2025_world')
        self.declare_parameter('spawn_delay', 5.0)

        self._preset_name = self.get_parameter('preset').value
        yaml_path = self.get_parameter('spawns_yaml').value
        self._world = self.get_parameter('world_name').value
        delay = float(self.get_parameter('spawn_delay').value)

        if not yaml_path:
            pkg = get_package_share_directory('atwork_arena_description')
            yaml_path = os.path.join(pkg, 'config', 'object_spawns.yaml')

        self.get_logger().info(f'Config: {yaml_path}')
        self.get_logger().info(f'Preset: {self._preset_name}')
        self.get_logger().info(f'World:  {self._world}')

        with open(yaml_path, 'r') as f:
            self._config = yaml.safe_load(f)

        self._tables = self._config.get('tables', {})
        self._drop_h = float(self._config.get('drop_height', 0.05))
        self._models_dir = os.path.join(
            get_package_share_directory('atwork_arena_description'), 'models')

        presets = self._config.get('presets', {})
        if self._preset_name not in presets:
            available = ', '.join(presets.keys())
            self.get_logger().error(
                f"Preset '{self._preset_name}' not found. Available: {available}")
            return

        self._preset = presets[self._preset_name]
        self.get_logger().info(
            f"Description: {self._preset.get('description', '—')}")
        self.get_logger().info(
            f"Objects to spawn: {len(self._preset.get('objects', []))}")
        self.get_logger().info(f'Spawning in {delay:.0f}s...')

        self._spawn_timer = self.create_timer(delay, self._do_spawn_async)

    def _do_spawn_async(self):
        self._spawn_timer.cancel()
        threading.Thread(target=self._do_spawn, daemon=True).start()

    def _do_spawn(self):
        objects = self._preset.get('objects', [])
        ok_count = 0
        fail_count = 0

        for obj in objects:
            if self._spawn_one(obj):
                ok_count += 1
            else:
                fail_count += 1
            time.sleep(0.5)

        self.get_logger().info(
            f'Done: {ok_count} spawned, {fail_count} failed '
            f'(preset={self._preset_name})')

    def _spawn_one(self, obj: dict) -> bool:
        name = obj['name']
        model_type = obj['type']
        table_name = obj['table']
        dx = float(obj.get('dx', 0.0))
        dy = float(obj.get('dy', 0.0))

        table = self._tables.get(table_name)
        if table is None:
            self.get_logger().warn(f"[{name}] table '{table_name}' not found")
            return False

        x = float(table['x']) + dx
        y = float(table['y']) + dy
        z = float(table['z_surface']) + self._drop_h

        sdf_path = os.path.join(self._models_dir, model_type, 'model.sdf')
        if not os.path.isfile(sdf_path):
            self.get_logger().warn(f"[{name}] model not found: {sdf_path}")
            return False

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self._world,
            '-file', sdf_path,
            '-name', name,
            '-allow_renaming', 'true',
            '-x', f'{x:.4f}',
            '-y', f'{y:.4f}',
            '-z', f'{z:.4f}',
        ]

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=15.0)
            if result.returncode == 0:
                self.get_logger().info(
                    f'  [{name}] {model_type} on {table_name} '
                    f'({x:.2f}, {y:.2f}, {z:.2f})')
                return True
            else:
                err = result.stderr.strip() or result.stdout.strip()
                self.get_logger().warn(f'  [{name}] failed: {err[:120]}')
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'  [{name}] spawn timed out')
            return False
        except Exception as e:
            self.get_logger().warn(f'  [{name}] error: {e}')
            return False


def main():
    rclpy.init()
    node = ObjectSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
