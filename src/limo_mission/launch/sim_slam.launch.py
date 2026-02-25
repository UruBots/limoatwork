"""
"""
sim_slam.launch.py — Simulation + SLAM + RViz to generate a map

Launches:
  1. Gazebo + robot (atwork_2024) with /scan, /odom, /cmd_vel
  2. slam_toolbox in mapping mode (publishes /map)
  3. RViz (map + LaserScan; teleop is run separately in another terminal)

Usage:
  # Terminal 1:
  ros2 launch limo_mission sim_slam.launch.py

  # Terminal 2 (when simulation is ready): move the robot
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

  # When the map is complete in RViz, save it:
  ros2 run nav2_map_server map_saver_cli -f /tmp/mapa_sim

Note: For the position in RViz to match Gazebo:
  - The map origin is set by SLAM with the first scan (robot must be at (0,0) in odom).
  - Do not move the robot until SLAM has been active for a few seconds (~10 s after launch).
  - odom_to_tf_node publishes odom→base_footprint from /odom; if the /tf bridge publishes
    another transform with different frames, the pose in RViz may drift.
"""
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_package_share_directory_or_raise(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{package_name}' not found. Run colcon build and source install/setup.bash."
        )


def generate_launch_description():
    pkg_limo_mission = _get_package_share_directory_or_raise("limo_mission")
    pkg_simulation = _get_package_share_directory_or_raise("limo_manipulator_bringup")

    sim_launch_path = os.path.join(pkg_simulation, "launch", "simulation.launch.py")
    if not os.path.isfile(sim_launch_path):
        raise FileNotFoundError(f"Simulation launch not found: {sim_launch_path}")

    # FastDDS: avoids communication issues in simulation
    fastdds_config = os.path.join(pkg_limo_mission, "config", "fastdds_no_shm.xml")
    set_fastdds = SetEnvironmentVariable(
        name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_config
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # --- Step 1: Simulation (Gazebo + robot); no /tf bridge, we use odom_to_tf ---
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bridge_tf": "false",
        }.items(),
    )

    # --- Odometry: publish TF odom → base_footprint from /odom (/tf bridge can fail) ---
    odom_to_tf_node = Node(
        package="limo_mission",
        executable="odom_to_tf_node",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- Step 2: SLAM (slam_toolbox in mapping mode; sim config with base_footprint) ---
    slam_params = os.path.join(pkg_limo_mission, "config", "slam_sim_toolbox.yaml")
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )
    # Start SLAM early so the first scan happens with the robot still at the origin;
    # this way the "map" frame is aligned with the Gazebo world.
    delayed_slam = TimerAction(period=5.0, actions=[slam_toolbox_node])

    # --- RViz (map + LaserScan; config with RViz2 classes) ---
    rviz_config = os.path.join(pkg_limo_mission, "config", "sim_slam.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    # Wait for SLAM to publish /map
    delayed_rviz = TimerAction(period=20.0, actions=[rviz_node])

    return LaunchDescription([
        set_fastdds,
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        simulation,
        odom_to_tf_node,
        delayed_slam,
        delayed_rviz,
    ])
