"""
mission_2025.launch.py -- Full simulation stack for RoboCup@Work 2025 Salvador

Launches:
  1. Gazebo + robot (atwork_2025 world, selectable arm)
  2. Object spawner (places manipulation objects on tables per test_mode preset)
  3. Odometry TF bridge
  4. Nav2 navigation stack (AMCL + planner + controller)
  5. Manipulation stack (object_detector + manipulation_manager + mycobot)
  6. Mission Manager 2025 (state-machine node)
  7. RViz (optional)

Usage:
  ros2 launch limo_mission mission_2025.launch.py
  ros2 launch limo_mission mission_2025.launch.py arm:=mycobot
  ros2 launch limo_mission mission_2025.launch.py enable_manipulation:=true
  ros2 launch limo_mission mission_2025.launch.py test_mode:=BTT1 spawn_objects:=true
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{name}' not found. Run colcon build and source install/setup.bash."
        )


def generate_launch_description():
    pkg_mission = _pkg("limo_mission")
    pkg_bringup = _pkg("limo_bringup")
    pkg_simulation = _pkg("limo_manipulator_bringup")

    try:
        pkg_manipulation = _pkg("limo_manipulation")
        manipulation_available = True
    except RuntimeError:
        manipulation_available = False

    sim_launch = os.path.join(pkg_simulation, "launch", "simulation.launch.py")
    nav_launch = os.path.join(pkg_bringup, "launch", "limo_start_navigation.launch.py")
    sim_nav_params = os.path.join(pkg_mission, "config", "sim_nav_params.yaml")

    fastdds_config = os.path.join(pkg_mission, "config", "fastdds_no_shm.xml")
    set_fastdds = SetEnvironmentVariable(
        name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_config)

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    enable_manipulation = LaunchConfiguration("enable_manipulation")
    start_rviz = LaunchConfiguration("start_rviz")
    task_yaml = LaunchConfiguration("task_yaml")
    arm = LaunchConfiguration("arm")
    world = LaunchConfiguration("world")
    test_mode = LaunchConfiguration("test_mode")
    spawn_objects = LaunchConfiguration("spawn_objects")
    spawn_preset = LaunchConfiguration("spawn_preset")

    # 1. Gazebo + robot (2025 world, selectable arm)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bridge_tf": "false",
            "world": world,
            "arm": arm,
        }.items(),
    )

    # 1.5 Object spawner (places manipulation objects on tables)
    object_spawner = Node(
        condition=IfCondition(spawn_objects),
        package="limo_mission",
        executable="object_spawner",
        name="object_spawner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "preset": spawn_preset,
            "world_name": "atwork_2025_world",
            "spawn_delay": 15.0,
        }],
    )

    # 1.6 Odometry TF bridge
    odom_to_tf = Node(
        package="limo_mission",
        executable="odom_to_tf_node",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Navigation (Nav2 + AMCL)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": sim_nav_params,
            "autostart": "true",
        }.items(),
    )

    # 3. Manipulation stack (conditional)
    manipulation_actions = []
    if manipulation_available:
        manip_launch = os.path.join(
            pkg_manipulation, "launch", "manipulation.launch.py")
        manipulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manip_launch),
            condition=IfCondition(enable_manipulation),
            launch_arguments={
                "use_sim": use_sim_time,
                "enable_mycobot": "true",
                "use_pymycobot": "false",
                "visualize": "false",
            }.items(),
        )
        manipulation_actions.append(
            TimerAction(period=40.0, actions=[manipulation]))

    # 4. Mission Manager 2025
    mission_manager = Node(
        package="limo_mission",
        executable="mission_manager_2025",
        name="mission_manager_2025",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "waypoints_yaml": os.path.join(
                pkg_mission, "config", "waypoints_2025.yaml"),
            "task_yaml": task_yaml,
            "test_mode": test_mode,
            "scan_topic": "/scan",
            "camera_topic": "/camera/image_raw",
            "initial_pose_x": -3.5,
            "initial_pose_y": 2.8,
            "initial_pose_yaw": 0.0,
            "nav_timeout_sec": 600.0,
            "nav2_startup_timeout_sec": 180.0,
            "nav_retries": 3,
            "skip_docking": False,
            "dock_timeout_sec": 15.0,
            "dock_retries": 2,
            "backup_distance": 0.20,
            "backup_speed": 0.10,
            "backup_timeout_sec": 6.0,
            "enable_manipulation": enable_manipulation,
            "manipulation_timeout_sec": 60.0,
            "detect_timeout_sec": 10.0,
            "primary_arm": "open_manipulator",
            "secondary_arm": "mycobot",
            "primary_arm_ns": "/manipulation",
            "secondary_arm_ns": "/mycobot",
            "primary_manager_node": "/manipulation_manager",
            "secondary_manager_node": "/mycobot_manager",
            "enable_tape_detection": True,
        }],
    )

    # 5. RViz
    rviz_config = os.path.join(pkg_mission, "config", "sim_slam.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mission_2025",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(start_rviz),
    )

    delayed_nav = TimerAction(period=20.0, actions=[navigation])
    delayed_rviz = TimerAction(period=25.0, actions=[rviz])
    delayed_mission = TimerAction(period=65.0, actions=[mission_manager])

    return LaunchDescription([
        set_fastdds,
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_manipulation", default_value="false",
                              description="Enable dual-arm pick/place services"),
        DeclareLaunchArgument("start_rviz", default_value="true",
                              description="Launch RViz visualization"),
        DeclareLaunchArgument("task_yaml", default_value="",
                              description="Path to task definition YAML (empty = default patrol)"),
        DeclareLaunchArgument("arm", default_value="none",
                              choices=["open_manipulator", "mycobot", "none"],
                              description="Arm to mount on the LIMO (requires open_manipulator_x_description or mycobot_description unless 'none')"),
        DeclareLaunchArgument("world", default_value="atwork_2025.world",
                              description="World file from atwork_arena_description/worlds/"),
        DeclareLaunchArgument("map", default_value=os.path.join(
                              pkg_bringup, "maps", "map.yaml"),
                              description="Path to Nav2 map YAML file"),
        DeclareLaunchArgument("test_mode", default_value="",
                              description="Competition test: BMT, BTT1, BTT2, ATT1, ATT2, FINAL"),
        DeclareLaunchArgument("spawn_objects", default_value="true",
                              description="Spawn manipulation objects on tables"),
        DeclareLaunchArgument("spawn_preset", default_value="BTT1",
                              description="Object spawn preset: BMT, BTT1, BTT2, ATT1, ATT2, FINAL"),
        simulation,
        object_spawner,
        odom_to_tf,
        delayed_nav,
        delayed_rviz,
        delayed_mission,
    ] + manipulation_actions)
