import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction,
                             OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_share_or_none(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _setup(context, *args, **kwargs):
    """Resolve launch arguments and build nodes that depend on string values."""
    arm_type = LaunchConfiguration('arm').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = []

    if arm_type != 'none':
        controllers = ['joint_state_broadcaster']
        if arm_type == 'open_manipulator':
            controllers += ['arm_controller', 'gripper_controller']
        elif arm_type == 'mycobot':
            controllers += ['mycobot_arm_controller']

        load_controllers = Node(
            package='controller_manager',
            executable='spawner',
            arguments=controllers + ['--switch-timeout', '30'],
            output='screen',
        )
        actions.append(TimerAction(period=35.0, actions=[load_controllers]))

    return actions


def generate_launch_description():
    pkg_limo_manipulator_description = get_package_share_directory(
        'limo_manipulator_description')
    pkg_atwork_arena_description = get_package_share_directory(
        'atwork_arena_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    resource_dirs = [
        os.path.join(pkg_atwork_arena_description, '..'),
        os.path.join(pkg_atwork_arena_description, 'models'),
        os.path.join(pkg_limo_manipulator_description, '..'),
    ]
    for optional_pkg in ['open_manipulator_x_description',
                         'open_manipulator_description',
                         'limo_description',
                         'mycobot_description']:
        pkg_dir = _pkg_share_or_none(optional_pkg)
        if pkg_dir:
            resource_dirs.append(os.path.join(pkg_dir, '..'))

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_dirs),
    )

    gz_relax_check = SetEnvironmentVariable(
        name='GZ_SIM_RELAX_VERSION_CHECK', value='1')

    gz_control_lib = _pkg_share_or_none('gz_ros2_control')
    plugin_dirs = []
    if gz_control_lib:
        plugin_dirs.append(os.path.join(gz_control_lib, '..', 'lib'))
    existing = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if existing:
        plugin_dirs.append(existing)
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=':'.join(plugin_dirs),
    )

    # ── Launch arguments ──
    use_sim_time = LaunchConfiguration('use_sim_time')
    bridge_tf = LaunchConfiguration('bridge_tf')
    world = LaunchConfiguration('world')
    arm = LaunchConfiguration('arm')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # ── Robot description (xacro with arm_type argument) ──
    xacro_file = os.path.join(
        pkg_limo_manipulator_description, 'urdf', 'limo_manipulator.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' arm_type:=', arm]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # ── Gazebo Sim ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                os.path.join(pkg_atwork_arena_description, 'worlds'), '/',
                world,
            ],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Spawn robot ──
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ── Gazebo ↔ ROS 2 bridges ──
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    bridge_tf_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(bridge_tf),
    )

    return LaunchDescription([
        gz_sim_resource_path,
        gz_relax_check,
        gz_plugin_path,

        # ── Declare arguments ──
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'bridge_tf', default_value='true',
            description='Bridge /tf from Gazebo; set false for SLAM with '
                        'odom_to_tf'),
        DeclareLaunchArgument(
            'world', default_value='atwork_2025.world',
            description='World file from atwork_arena_description/worlds/'),
        DeclareLaunchArgument(
            'arm', default_value='mycobot',
            choices=['open_manipulator', 'mycobot', 'none'],
            description='Arm to mount: open_manipulator, mycobot, or none'),
        DeclareLaunchArgument('x_pose', default_value='-3.5',
                              description='Robot spawn X (default: START area)'),
        DeclareLaunchArgument('y_pose', default_value='2.8',
                              description='Robot spawn Y (default: START area)'),
        DeclareLaunchArgument('z_pose', default_value='0.0'),

        # ── Nodes ──
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        bridge_tf_node,

        # ── Controllers (arm-dependent, resolved at launch time) ──
        OpaqueFunction(function=_setup),
    ])
