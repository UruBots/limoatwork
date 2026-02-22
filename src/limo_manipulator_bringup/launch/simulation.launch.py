import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Paths
    pkg_limo_manipulator_description = get_package_share_directory('limo_manipulator_description')
    pkg_atwork_arena_description = get_package_share_directory('atwork_arena_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo Sim Resource Path
    # Each entry must be the PARENT of the named package folder so that
    # model://package_name/... URIs can be resolved by Gazebo.
    # e.g. model://atwork_arena_description/materials/textures/foo.png
    #   → <entry>/atwork_arena_description/materials/textures/foo.png
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            # Parent of atwork_arena_description — resolves model://atwork_arena_description/...
            os.path.join(pkg_atwork_arena_description, '..'),
            ':',
            # Direct models folder (kept for backwards-compatible bare model names)
            os.path.join(pkg_atwork_arena_description, 'models'),
            ':',
            os.path.join(pkg_limo_manipulator_description, '..'),
            ':',
            os.path.join(get_package_share_directory('open_manipulator_description'), '..'),
            ':',
            os.path.join(get_package_share_directory('limo_description'), '..')
        ]
    )

    # Allow Gazebo Harmonic to load Ignition plugins if needed
    gz_relax_check = SetEnvironmentVariable(
        name='GZ_SIM_RELAX_VERSION_CHECK',
        value='1'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Robot Description (Xacro)
    xacro_file = os.path.join(pkg_limo_manipulator_description, 'urdf', 'limo_manipulator.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(get_package_share_directory('atwork_arena_description'), 'worlds', 'atwork_2024.world'),
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'limo_manipulator',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Cmd Vel
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Joint States
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # RGB Camera image (for AprilTag detection)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Camera info (intrinsics, needed for pose estimation)
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Single spawner activating all controllers as a group with a generous
    # --switch-timeout.  gz_ros2_control (Ignition Fortress) has a hard-coded
    # 5-second default that is too short when Gazebo physics is catching up at
    # startup.  30 seconds gives enough headroom without blocking forever.
    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'arm_controller',
            'gripper_controller',
            '--switch-timeout', '30',
        ],
        output='screen',
    )

    # Static TF: bridge the Ignition sensor frame to the URDF laser_link frame.
    # When ros_gz_bridge forwards /scan, Ignition uses the collapsed SDF path as
    # frame_id (limo_manipulator/base_footprint/laser_sensor). This static TF
    # makes that frame a child of laser_link so AMCL can look up the transform.
    laser_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0',
                   'laser_link',
                   'limo_manipulator/base_footprint/laser_sensor'],
        output='screen',
    )

    # Delay the group spawner enough for gz_ros2_control to fully initialize.
    # Spawning at t=35 s (before nav2 starts its TF initialization at t=20+X s)
    # avoids the "jump back in time" clock artifact that causes configure failures.
    delayed_controllers = TimerAction(
        period=35.0,
        actions=[load_controllers],
    )

    return LaunchDescription([
        gz_sim_resource_path,
        gz_relax_check,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        laser_frame_bridge,
        delayed_controllers,
    ])
