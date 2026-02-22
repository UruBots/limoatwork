import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_pkg(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{name}' not found. Run 'colcon build' and source install/setup.bash."
        )


def generate_launch_description():
    pkg_limo_mission  = _get_pkg('limo_mission')
    pkg_limo_bringup  = _get_pkg('limo_bringup')
    pkg_manipulation  = _get_pkg('limo_manipulation')

    base_launch_path        = os.path.join(pkg_limo_bringup,  'launch', 'limo_start.launch.py')
    nav_launch_path         = os.path.join(pkg_limo_bringup,  'launch', 'navigation2.launch.py')
    manipulation_launch_path = os.path.join(pkg_manipulation,  'launch', 'manipulation.launch.py')

    if not os.path.isfile(base_launch_path):
        raise FileNotFoundError(f"Base launch not found: {base_launch_path}")
    if not os.path.isfile(nav_launch_path):
        raise FileNotFoundError(f"Navigation launch not found: {nav_launch_path}")

    # Launch configurations
    map_file   = LaunchConfiguration('map',
                     default=os.path.join(pkg_limo_bringup, 'maps', 'map.yaml'))
    route_name = LaunchConfiguration('route_name', default='test_route')

    # 1. Robot base (limo_base + lidar + filters)
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path)
    )

    # 2. Navigation stack Nav2 + AMCL (uses amcl_params.yaml by default)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false'
        }.items()
    )

    # 3. Manipulation stack (RealSense + object_detector + manipulation_manager)
    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manipulation_launch_path),
        launch_arguments={'use_sim': 'false'}.items()
    )

    # 4. Mission Manager
    mission_manager = Node(
        package='limo_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan_filtered',
            'route_name': route_name,
            'waypoints_yaml': os.path.join(pkg_limo_mission, 'config', 'waypoints.yaml'),
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value=os.path.join(pkg_limo_bringup, 'maps', 'map.yaml'),
            description='Full path to the .yaml map file'),
        DeclareLaunchArgument('route_name',
            default_value='test_route',
            description='Route name defined in waypoints.yaml'),
        base,
        navigation,
        manipulation,
        mission_manager,
    ])
