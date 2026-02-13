import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    port_name = DeclareLaunchArgument(
        name='port_name',
        default_value='ttyUSB0'
    )
    odom_topic_name = DeclareLaunchArgument(
        name='odom_topic_name',
        default_value='odom'
    )
    open_rviz = DeclareLaunchArgument(
        name='open_rviz',
        default_value='false'
    )

    # Frame do lidar (ajuste se o teu LaserScan vier com outro frame_id)
    lidar_frame = DeclareLaunchArgument(
        name='lidar_frame',
        default_value='laser_frame'
    )

    rviz_node = Node(
        package='rviz2',
        name='rviz2',
        executable='rviz',
        on_exit='kill',
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('open_rviz')
        )
    )

    # base_link -> imu_link (como você já tinha)
    static_transform_publisher_imu_node = Node(
        package='tf2_ros',
        name='static_transform_publisher_imu',
        executable='static_transform_publisher',
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "imu_link"]
    )

    # base_link -> lidar_frame (12 cm no X)
    static_transform_publisher_lidar_node = Node(
        package='tf2_ros',
        name='static_transform_publisher_lidar',
        executable='static_transform_publisher',
        arguments=[
            "0.12", "0.0", "0.0",   # x y z (m)
            "0.0", "0.0", "0.0",    # roll pitch yaw (rad)
            "base_link",
            LaunchConfiguration('lidar_frame')
        ]
    )

    limo_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('limo_base'),
                'launch/limo_base.launch.py'
            )
        ),
        launch_arguments={
            'port_name': LaunchConfiguration('port_name'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name')
        }.items()
    )

    urg_node2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('urg_node2'),
                'launch/urg_node2.launch.py'
            )
        ),
        launch_arguments={'publish_tf': 'false'}.items()
    )

    # -----------------------------
    # LaserScan FILTER chain node
    # -----------------------------
    BRINGUP_PKG = 'limo_bringup'  # TROQUE se teu pacote de bringup tiver outro nome

    laser_filters_yaml = os.path.join(
        get_package_share_directory(BRINGUP_PKG),
        'config_files',
        'laser_filters.yaml'
    )

    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter_chain',
        output='screen',
        parameters=[laser_filters_yaml],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    ld = LaunchDescription([
        port_name,
        odom_topic_name,
        open_rviz,
        lidar_frame,

        rviz_node,

        static_transform_publisher_imu_node,
        static_transform_publisher_lidar_node,

        limo_base_launch,
        urg_node2_launch,

        scan_filter_node,

        launch.actions.LogInfo(msg="LIMO Lidar: urg_node2 started. Raw scan in /scan."),
        launch.actions.LogInfo(msg="TF: base_link -> lidar at x=0.12m (set by static_transform_publisher)."),
        launch.actions.LogInfo(msg="Laser filters: publishing filtered scan in /scan_filtered (use this for Cartographer/Nav2)."),
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
