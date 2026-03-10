#!/usr/bin/env python3
"""
manipulation.launch.py
=======================
Launches the full manipulation stack for LIMO + arm (OpenManipulator-X or MyCobot 280).

Supports two modes via the 'use_sim' argument:

  use_sim:=true   (default)
    - Assumes Gazebo is already running (launched by simulation_mission.launch.py).
    - Assumes arm_controller is already spawned.
    - Launches: MoveIt2 move_group (optional) + object_detector + manipulation_manager.
    - Does NOT launch RealSense driver.

  use_sim:=false  (real robot)
    - Assumes limo_start.launch.py is already running (base).
    - Launches: RealSense driver + MoveIt2 + object_detector + manipulation_manager.

Usage (simulation -- MyCobot with MoveIt2):
  ros2 launch limo_manipulation manipulation.launch.py arm:=mycobot use_moveit:=true

Usage (simulation -- OpenManipulator-X):
  ros2 launch limo_manipulation manipulation.launch.py arm:=open_manipulator use_moveit:=true

Usage (real robot):
  ros2 launch limo_manipulation manipulation.launch.py use_sim:=false
"""

import os
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ====================================================================== #
#  Helpers                                                                #
# ====================================================================== #

def _load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def _build_mycobot_moveit_params(pkg_manipulation, pkg_manipulator_desc):
    """Build MoveIt2 parameter dict for MyCobot 280 on LIMO."""
    cfg_dir = os.path.join(pkg_manipulation, 'config', 'mycobot_moveit')
    xacro_file = os.path.join(
        pkg_manipulator_desc, 'urdf', 'limo_manipulator.xacro')
    robot_description_xml = xacro.process_file(
        xacro_file, mappings={'arm_type': 'mycobot'}).toxml()

    with open(os.path.join(cfg_dir, 'limo_manipulator.srdf'), 'r') as f:
        srdf_xml = f.read()

    kinematics = _load_yaml(os.path.join(cfg_dir, 'kinematics.yaml'))
    joint_limits = _load_yaml(os.path.join(cfg_dir, 'joint_limits.yaml'))
    controllers = _load_yaml(os.path.join(cfg_dir, 'moveit_controllers.yaml'))
    pilz = _load_yaml(os.path.join(cfg_dir, 'pilz_cartesian_limits.yaml'))

    return {
        'robot_description': robot_description_xml,
        'robot_description_semantic': srdf_xml,
        'robot_description_kinematics': kinematics,
        'robot_description_planning': joint_limits,
        **controllers,
        'pilz_cartesian_limits': pilz,
    }


# ====================================================================== #
#  OpaqueFunction: resolves MoveIt2 nodes based on 'arm' argument        #
# ====================================================================== #

def _setup_moveit_nodes(context):
    """Called at launch-time to create MoveIt2 nodes for the selected arm."""
    arm_type = context.launch_configurations['arm']
    use_sim = context.launch_configurations.get('use_sim', 'true')
    use_moveit = context.launch_configurations.get('use_moveit', 'false')
    start_rviz = context.launch_configurations.get('start_rviz', 'false')

    if use_moveit.lower() != 'true':
        return []

    pkg_manipulation = get_package_share_directory('limo_manipulation')
    pkg_manipulator_desc = get_package_share_directory(
        'limo_manipulator_description')

    nodes = []

    if arm_type == 'mycobot':
        params = _build_mycobot_moveit_params(
            pkg_manipulation, pkg_manipulator_desc)

        nodes.append(Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                params,
                {'use_sim_time': use_sim == 'true'},
            ],
        ))

        if start_rviz.lower() == 'true':
            nodes.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_manipulation',
                output='log',
                arguments=['-d', os.path.join(
                    pkg_manipulation, 'config', 'manipulation_rviz.rviz')],
                parameters=[
                    {'robot_description':
                        params['robot_description']},
                    {'robot_description_semantic':
                        params['robot_description_semantic']},
                    {'robot_description_kinematics':
                        params['robot_description_kinematics']},
                    {'robot_description_planning':
                        params['robot_description_planning']},
                    {'use_sim_time': use_sim == 'true'},
                ],
            ))

    elif arm_type == 'open_manipulator':
        try:
            from moveit_configs_utils import MoveItConfigsBuilder
            get_package_share_directory('open_manipulator_moveit_config')
        except Exception:
            return []

        moveit_config = (
            MoveItConfigsBuilder(
                robot_name='open_manipulator_x',
                package_name='open_manipulator_moveit_config',
            )
            .robot_description_semantic(
                str(Path('config') / 'open_manipulator_x'
                    / 'open_manipulator_x.srdf'))
            .joint_limits(
                str(Path('config') / 'open_manipulator_x'
                    / 'joint_limits.yaml'))
            .trajectory_execution(
                str(Path('config') / 'open_manipulator_x'
                    / 'moveit_controllers.yaml'))
            .robot_description_kinematics(
                str(Path('config') / 'open_manipulator_x'
                    / 'kinematics.yaml'))
            .to_moveit_configs()
        )

        nodes.append(Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim == 'true'},
            ],
        ))

        if start_rviz.lower() == 'true':
            nodes.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_manipulation',
                output='log',
                arguments=['-d', os.path.join(
                    pkg_manipulation, 'config', 'manipulation_rviz.rviz')],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {'use_sim_time': use_sim == 'true'},
                ],
            ))

    return nodes


# ====================================================================== #
#  generate_launch_description                                            #
# ====================================================================== #

def generate_launch_description():
    pkg_limo_manipulation = get_package_share_directory('limo_manipulation')

    try:
        pkg_realsense = get_package_share_directory('realsense2_camera')
    except Exception:
        pkg_realsense = ''

    try:
        pkg_apriltag_detector = get_package_share_directory('apriltag_detector')
    except Exception:
        pkg_apriltag_detector = ''

    # ------------------------------------------------------------------ #
    #  Launch arguments                                                   #
    # ------------------------------------------------------------------ #
    arm_arg = DeclareLaunchArgument(
        'arm', default_value='mycobot',
        description='Arm type: "mycobot" or "open_manipulator"')
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='true = Gazebo simulation, false = real robot')
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit', default_value='false',
        description='Launch MoveIt2 move_group node')
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz', default_value='false',
        description='Launch RViz with MoveIt2 motion planning plugin')
    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='true',
        description='Launch the OpenCV detection dashboard')
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='false',
        description='Open a local cv2.imshow window')
    enable_mycobot_arg = DeclareLaunchArgument(
        'enable_mycobot', default_value='false',
        description='Launch the MyCobot 280 manager alongside OpenManipulator-X')
    use_pymycobot_arg = DeclareLaunchArgument(
        'use_pymycobot', default_value='true',
        description='MyCobot backend: true=pymycobot serial, false=ros2_control')
    mycobot_port_arg = DeclareLaunchArgument(
        'mycobot_port', default_value='/dev/ttyUSB0',
        description='Serial port for MyCobot')

    use_sim     = LaunchConfiguration('use_sim')
    visualize   = LaunchConfiguration('visualize')
    show_window = LaunchConfiguration('show_window')

    # ------------------------------------------------------------------ #
    #  Config paths                                                       #
    # ------------------------------------------------------------------ #
    arm_poses_yaml    = os.path.join(
        pkg_limo_manipulation, 'config', 'arm_poses.yaml')
    objects_cfg_yaml  = os.path.join(
        pkg_limo_manipulation, 'config', 'objects_config.yaml')
    apriltag_cfg_yaml = os.path.join(
        pkg_limo_manipulation, 'config', 'apriltag_config.yaml')

    # ------------------------------------------------------------------ #
    #  RealSense driver (real robot only)                                 #
    # ------------------------------------------------------------------ #
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
            if pkg_realsense else '/dev/null'
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'camera_name':      'camera',
            'camera_namespace': 'camera',
            'enable_color':     'true',
            'enable_depth':     'true',
            'align_depth.enable': 'true',
            'depth_module.depth_profile':  '640,480,30',
            'depth_module.color_profile':  '640,480,30',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    #  AprilTag detector                                                  #
    # ------------------------------------------------------------------ #
    apriltag_sim_node = Node(
        condition=IfCondition(use_sim),
        package='limo_manipulation',
        executable='apriltag_sim_publisher',
        name='apriltag_sim_publisher',
        output='screen',
        parameters=[{
            'apriltag_config':  apriltag_cfg_yaml,
            'objects_config':   objects_cfg_yaml,
            'camera_frame':     'camera_color_optical_frame',
            'publish_rate_hz':  10.0,
            'detection_range':  1.2,
            'fov_deg':          60.0,
        }],
    )

    apriltag_real_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_apriltag_detector, 'launch', 'detect.launch.py')
            if pkg_apriltag_detector else '/dev/null'
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'camera':      'camera',
            'image':       'color/image_raw',
            'tags':        '/apriltag/detections',
            'tag_family':  'tf36h11',
            'type':        'mit',
            'num_threads': '2',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    #  Object detector node                                               #
    # ------------------------------------------------------------------ #
    object_detector_node = Node(
        package='limo_manipulation',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim':         use_sim,
            'objects_config':  objects_cfg_yaml,
            'apriltag_config': apriltag_cfg_yaml,
            'camera_frame':    'camera_color_optical_frame',
            'target_frame':    'map',
            'tag_timeout_sec': 3.0,
        }],
    )

    # ------------------------------------------------------------------ #
    #  Manipulation manager node (OpenManipulator-X)                      #
    # ------------------------------------------------------------------ #
    manipulation_manager_node = Node(
        package='limo_manipulation',
        executable='manipulation_manager',
        name='manipulation_manager',
        output='screen',
        parameters=[{
            'use_sim':          use_sim,
            'arm_poses_config': arm_poses_yaml,
            'objects_config':   objects_cfg_yaml,
            'detect_timeout':   10.0,
            'motion_timeout':   30.0,
        }],
    )

    # ------------------------------------------------------------------ #
    #  MyCobot 280 manager node                                           #
    # ------------------------------------------------------------------ #
    mycobot_poses_yaml = os.path.join(
        pkg_limo_manipulation, 'config', 'mycobot_poses.yaml')
    enable_mycobot = LaunchConfiguration('enable_mycobot')

    mycobot_manager_node = Node(
        condition=IfCondition(enable_mycobot),
        package='limo_manipulation',
        executable='mycobot_manager',
        name='mycobot_manager',
        output='screen',
        parameters=[{
            'use_sim':               use_sim,
            'use_pymycobot':         LaunchConfiguration('use_pymycobot'),
            'port':                  LaunchConfiguration('mycobot_port'),
            'baud':                  115200,
            'mycobot_poses_config':  mycobot_poses_yaml,
            'objects_config':        objects_cfg_yaml,
            'detect_timeout':        10.0,
            'motion_timeout':        20.0,
        }],
    )

    # ------------------------------------------------------------------ #
    #  Detection visualizer (OpenCV dashboard)                           #
    # ------------------------------------------------------------------ #
    detection_visualizer_node = Node(
        condition=IfCondition(visualize),
        package='limo_manipulation',
        executable='detection_visualizer',
        name='detection_visualizer',
        output='screen',
        parameters=[{
            'use_sim':     use_sim,
            'show_window': show_window,
            'panel_width': 320,
            'font_scale':  0.55,
        }],
    )

    # ------------------------------------------------------------------ #
    #  Assembly                                                           #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        arm_arg,
        use_sim_arg,
        use_moveit_arg,
        start_rviz_arg,
        visualize_arg,
        show_window_arg,
        enable_mycobot_arg,
        use_pymycobot_arg,
        mycobot_port_arg,

        realsense_launch,

        apriltag_sim_node,
        apriltag_real_node,

        # MoveIt2 nodes resolved at runtime via OpaqueFunction
        OpaqueFunction(function=_setup_moveit_nodes),

        object_detector_node,
        manipulation_manager_node,

        mycobot_manager_node,

        detection_visualizer_node,
    ])
