"""
spawn_objects.launch.py -- Spawn manipulation objects in a running Gazebo simulation.

Standalone launch that places objects on tables without starting the full
mission stack. Useful for testing manipulation or quick arena setup.

Usage:
  ros2 launch limo_mission spawn_objects.launch.py
  ros2 launch limo_mission spawn_objects.launch.py preset:=ATT1
  ros2 launch limo_mission spawn_objects.launch.py preset:=FINAL spawn_delay:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("preset", default_value="BTT1",
                              description="Object preset: BMT, BTT1, BTT2, ATT1, ATT2, FINAL"),
        DeclareLaunchArgument("world_name", default_value="atwork_2025_world",
                              description="Gazebo world name (used for create service)"),
        DeclareLaunchArgument("spawn_delay", default_value="2.0",
                              description="Seconds to wait before spawning"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="limo_mission",
            executable="object_spawner",
            name="object_spawner",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "preset": LaunchConfiguration("preset"),
                "world_name": LaunchConfiguration("world_name"),
                "spawn_delay": LaunchConfiguration("spawn_delay"),
            }],
        ),
    ])
