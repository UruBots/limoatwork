#!/usr/bin/env python3
"""
mycobot.launch.py
=================
Launches the MyCobot 280 manipulation stack.

Modes
-----
  use_sim:=true   (default)
    Uses ros2_control with mock hardware.
    Assumes Gazebo & controllers are already spawned.

  use_sim:=false  (real robot)
    Uses pymycobot directly over serial.
    No ros2_control needed.

Usage:
  ros2 launch limo_manipulation mycobot.launch.py

  ros2 launch limo_manipulation mycobot.launch.py \
      use_sim:=false port:=/dev/ttyUSB0 baud:=115200
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('limo_manipulation')

    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='true = simulation, false = real robot')
    use_pymycobot_arg = DeclareLaunchArgument(
        'use_pymycobot', default_value='true',
        description='Use pymycobot library directly (real robot)')
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port for MyCobot')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='Baud rate for MyCobot serial')

    mycobot_poses_yaml = os.path.join(
        pkg_dir, 'config', 'mycobot_poses.yaml')
    objects_cfg_yaml = os.path.join(
        pkg_dir, 'config', 'objects_config.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_pymycobot = LaunchConfiguration('use_pymycobot')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    mycobot_manager_node = Node(
        package='limo_manipulation',
        executable='mycobot_manager',
        name='mycobot_manager',
        output='screen',
        parameters=[{
            'use_sim':               use_sim,
            'use_pymycobot':         use_pymycobot,
            'port':                  port,
            'baud':                  baud,
            'mycobot_poses_config':  mycobot_poses_yaml,
            'objects_config':        objects_cfg_yaml,
            'detect_timeout':        10.0,
            'motion_timeout':        20.0,
        }],
    )

    return LaunchDescription([
        use_sim_arg,
        use_pymycobot_arg,
        port_arg,
        baud_arg,
        mycobot_manager_node,
    ])
