import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _get_pkg(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Package '{name}' not found. Run 'colcon build' and source install/setup.bash."
        )


def generate_launch_description():
    pkg_limo_bringup = _get_pkg('limo_bringup')

    base_launch_path = os.path.join(pkg_limo_bringup, 'launch', 'limo_start.launch.py')
    carto_launch_path = os.path.join(pkg_limo_bringup, 'launch', 'cartographer.launch.py')

    if not os.path.isfile(base_launch_path):
        raise FileNotFoundError(f"Base launch not found: {base_launch_path}")
    if not os.path.isfile(carto_launch_path):
        raise FileNotFoundError(f"Cartographer launch not found: {carto_launch_path}")

    # 1. Base del robot (limo_base + lidar + filtros)
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path)
    )

    # 2. Cartographer SLAM
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(carto_launch_path),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        base,
        cartographer,
    ])
