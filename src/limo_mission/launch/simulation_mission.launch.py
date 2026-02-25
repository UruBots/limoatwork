import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                             TimerAction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_package_share_directory_or_raise(package_name):
    """Return package share path or raise a clear error."""
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{package_name}' not found in install space. "
            "Run 'colcon build' and source install/setup.bash for the full workspace."
        )


def generate_launch_description():
    # Resolve paths first so we fail fast with a clear error if any package is missing
    pkg_limo_mission = _get_package_share_directory_or_raise('limo_mission')
    pkg_limo_bringup = _get_package_share_directory_or_raise('limo_bringup')
    pkg_simulation = _get_package_share_directory_or_raise('limo_manipulator_bringup')
    pkg_manipulation = _get_package_share_directory_or_raise('limo_manipulation')

    sim_launch_path      = os.path.join(pkg_simulation,   'launch', 'simulation.launch.py')
    nav_launch_path      = os.path.join(pkg_limo_bringup, 'launch', 'limo_start_navigation.launch.py')
    manipulation_launch  = os.path.join(pkg_manipulation,  'launch', 'manipulation.launch.py')
    sim_nav_params       = os.path.join(pkg_limo_mission,  'config', 'sim_nav_params.yaml')
    if not os.path.isfile(sim_launch_path):
        raise FileNotFoundError(f"Simulation launch file not found: {sim_launch_path}")
    if not os.path.isfile(nav_launch_path):
        raise FileNotFoundError(f"Navigation launch file not found: {nav_launch_path}")
    if not os.path.isfile(sim_nav_params):
        raise FileNotFoundError(f"Simulation nav params not found: {sim_nav_params}")

    # FastDDS: disable shared memory to avoid inter-process communication failures
    # in simulation (Gazebo + many ROS2 nodes).
    fastdds_config = os.path.join(pkg_limo_mission, 'config', 'fastdds_no_shm.xml')
    set_fastdds = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_config)

    # Launch Configurations
    use_sim_time        = LaunchConfiguration('use_sim_time',        default='true')
    map_file            = LaunchConfiguration('map',                 default=os.path.join(pkg_limo_bringup, 'maps', 'map.yaml'))
    route_name          = LaunchConfiguration('route_name',          default='full_mission')
    enable_manipulation = LaunchConfiguration('enable_manipulation', default='false')
    start_rviz          = LaunchConfiguration('start_rviz',          default='true')

    # 1. Gazebo Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'bridge_tf': 'false'
        }.items()
    )

    # 1.5. Odometry: publish TF odom → base_footprint from /odom (/tf bridge can fail)
    odom_to_tf_node = Node(
        package="limo_mission",
        executable="odom_to_tf_node",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Navigation Stack (Nav2 + AMCL) — uses limo_mission/config/sim_nav_params.yaml
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': sim_nav_params,
            'autostart': 'true'
        }.items()
    )

    # 3. Mission Manager
    mission_manager = Node(
        package='limo_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',  # En simulación el lidar va directo a /scan
            'route_name': route_name,
            'waypoints_yaml': os.path.join(pkg_limo_mission, 'config', 'waypoints-sim.yaml'),
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,   # Gazebo spawn origin, FREE in the map
            'initial_pose_yaw': 0.0,
            # Simulation runs slowly; generous timeouts
            'nav_timeout_sec': 600.0,
            'nav2_startup_timeout_sec': 180.0,
            # No real workstation walls in simulation — skip LiDAR docking entirely
            # and proceed directly from Nav2 arrival to pick.
            'skip_docking': False,
            'dock_timeout_sec': 2.0,
            'dock_retries': 1,
            # After picking, back up 0.20 m. The arm is first folded to HOME
            # (see mission_manager._run_mission), so a short backup is enough to
            # clear the workstation's inflation zone.  0.50 m caused the robot
            # to back into the arena wall at y≈-1.11 when arriving near the
            # WS01 tolerance boundary (xy_goal_tolerance=0.25 m pushes worst-case
            # landing to y≈-0.99, still > 0.11 m clear of the lethal zone).
            'backup_distance': 0.20,
            'backup_speed': 0.10,
            'backup_timeout_sec': 6.0,
            # Passed from CLI: ros2 launch ... enable_manipulation:=true
            'enable_manipulation': enable_manipulation,
        }]
    )

    # 4. Manipulation stack (object_detector + manipulation_manager)
    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manipulation_launch),
        launch_arguments={'use_sim': 'true'}.items()
    )

    # 5. RViz (mapa, robot, láser; opcional con start_rviz:=false)
    rviz_config = os.path.join(pkg_limo_mission, 'config', 'sim_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mission',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(start_rviz),
    )
    delayed_rviz = TimerAction(period=25.0, actions=[rviz_node])

    # Delay Nav2 so Gazebo has time to publish odom TF before Nav2 tries to activate.
    # The DiffDrive plugin needs a few sim steps after robot spawn to publish odom→base_link.
    delayed_navigation = TimerAction(period=20.0, actions=[navigation])

    # Delay manipulation stack until after Gazebo, Nav2, and ALL controllers are active.
    # Single group spawner fires at t=35 s and typically finishes by t=40 s.
    # Wait until t=45 s to give extra margin.
    delayed_manipulation = TimerAction(period=45.0, actions=[manipulation])

    # Delay mission manager long enough for:
    #   - Nav2 lifecycle to activate ALL nodes (AMCL, planner, controller) → ~55 s
    #   - Manipulation stack (controllers, manipulation_manager) → ~50 s
    # Using 65 s gives a generous safety margin on slow machines.
    # mission_manager also waits internally for TF map→odom (AMCL) with a
    # nav2_startup_timeout_sec=180 s timeout, so even if AMCL is slow it will retry.
    delayed_mission = TimerAction(period=65.0, actions=[mission_manager])

    return LaunchDescription([
        set_fastdds,
        DeclareLaunchArgument('use_sim_time',        default_value='true'),
        DeclareLaunchArgument('route_name',          default_value='full_mission'),
        DeclareLaunchArgument('enable_manipulation', default_value='false',
                              description='If true, call pick/place services at each workstation'),
        DeclareLaunchArgument('start_rviz', default_value='true',
                              description='If true, launch RViz with map/robot/laser config'),
        simulation,
        odom_to_tf_node,
        delayed_navigation,
        delayed_manipulation,
        delayed_rviz,
        delayed_mission,
    ])
