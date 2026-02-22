#!/usr/bin/env python3
"""
manipulation.launch.py
=======================
Launches the full manipulation stack for LIMO + OpenManipulator-X.

Supports two modes via the 'use_sim' argument:

  use_sim:=true   (default)
    - Assumes Gazebo is already running (launched by simulation_mission.launch.py).
    - Assumes arm_controller / gripper_controller are already spawned.
    - Launches: MoveIt2 move_group (optional) + object_detector + manipulation_manager.
    - Does NOT launch RealSense driver.

  use_sim:=false  (real robot)
    - Assumes limo_start.launch.py is already running (base).
    - Launches: RealSense driver + MoveIt2 + object_detector + manipulation_manager.

Usage (simulation):
  ros2 launch limo_manipulation manipulation.launch.py use_sim:=true

Usage (real robot):
  ros2 launch limo_manipulation manipulation.launch.py use_sim:=false

Both cases can be integrated into the mission launch file with:
  IncludeLaunchDescription('manipulation.launch.py',
      launch_arguments={'use_sim': 'true'}.items())
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_limo_manipulation = get_package_share_directory('limo_manipulation')
    pkg_moveit_config = get_package_share_directory('open_manipulator_moveit_config')
    pkg_limo_manipulator_desc = get_package_share_directory('limo_manipulator_description')

    # Optional packages — only available on the real robot.
    # Wrapped in try/except so simulation launches don't fail if they are absent.
    try:
        pkg_realsense = get_package_share_directory('realsense2_camera')
    except Exception:
        pkg_realsense = ''   # not installed; IncludeLaunchDescription guarded by UnlessCondition

    try:
        pkg_apriltag_detector = get_package_share_directory('apriltag_detector')
    except Exception:
        pkg_apriltag_detector = ''   # not installed; node guarded by UnlessCondition

    # ------------------------------------------------------------------ #
    #  Launch arguments                                                   #
    # ------------------------------------------------------------------ #
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='true = Gazebo simulation, false = real robot'
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Launch MoveIt2 move_group node (optional, for Cartesian planning)'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Launch RViz with MoveIt2 motion planning plugin'
    )
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description=(
            'Launch the OpenCV detection dashboard. '
            'Publishes /manipulation/debug_image and opens a window '
            '(set show_window:=false to publish only, without a local display).'
        )
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description=(
            'Open a local cv2.imshow window. '
            'Defaults to false to avoid the ~17%% CPU overhead of OpenCV GUI rendering. '
            'Use rqt_image_view /manipulation/debug_image instead. '
            'Set true only when you need the local window.'
        )
    )

    use_sim     = LaunchConfiguration('use_sim')
    use_moveit  = LaunchConfiguration('use_moveit')
    start_rviz  = LaunchConfiguration('start_rviz')
    visualize   = LaunchConfiguration('visualize')
    show_window = LaunchConfiguration('show_window')

    # ------------------------------------------------------------------ #
    #  Config paths                                                       #
    # ------------------------------------------------------------------ #
    arm_poses_yaml    = os.path.join(pkg_limo_manipulation, 'config', 'arm_poses.yaml')
    objects_cfg_yaml  = os.path.join(pkg_limo_manipulation, 'config', 'objects_config.yaml')
    apriltag_cfg_yaml = os.path.join(pkg_limo_manipulation, 'config', 'apriltag_config.yaml')

    # ------------------------------------------------------------------ #
    #  MoveIt2 move_group (optional – needed for Cartesian planning)      #
    #                                                                     #
    #  The MoveItConfigsBuilder reads the SRDF and kinematics from the   #
    #  open_manipulator_moveit_config package.  The robot_description    #
    #  (full limo_manipulator URDF) must already be on the parameter     #
    #  server (published by robot_state_publisher in simulation.launch). #
    # ------------------------------------------------------------------ #
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='open_manipulator_x',
            package_name='open_manipulator_moveit_config',
        )
        .robot_description_semantic(
            str(Path('config') / 'open_manipulator_x' / 'open_manipulator_x.srdf')
        )
        .joint_limits(
            str(Path('config') / 'open_manipulator_x' / 'joint_limits.yaml')
        )
        .trajectory_execution(
            str(Path('config') / 'open_manipulator_x' / 'moveit_controllers.yaml')
        )
        .robot_description_kinematics(
            str(Path('config') / 'open_manipulator_x' / 'kinematics.yaml')
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        condition=IfCondition(use_moveit),
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim},
        ],
    )

    # The manipulation RViz config shows: robot model, TF, laser scan,
    # the debug composite image (/manipulation/debug_image), and odometry.
    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_manipulation',
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('limo_manipulation'),
                'config', 'manipulation_rviz.rviz',
            ]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim},
        ],
    )

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
    #  AprilTag detector — runs on camera images                          #
    #                                                                     #
    #  Simulation:  apriltag_sim_publisher                                #
    #    Publishes synthetic /apriltag/detections from known world        #
    #    positions (no image needed, reliable for pipeline testing).      #
    #                                                                     #
    #  Real robot:  apriltag_detector (composable node, MIT backend)      #
    #    Runs the real MIT/umich AprilTag detector on RealSense images.   #
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
    #  Manipulation manager node                                          #
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
    #  Detection visualizer (OpenCV dashboard)                           #
    #                                                                    #
    #  Subscribes to camera, AprilTag detections, manipulation status    #
    #  and joint states.  Produces an annotated composite image on       #
    #  /manipulation/debug_image (viewable in rqt_image_view / RViz2)   #
    #  and optionally an interactive cv2.imshow window.                  #
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
        use_sim_arg,
        use_moveit_arg,
        start_rviz_arg,
        visualize_arg,
        show_window_arg,

        # RealSense (real robot only)
        realsense_launch,

        # AprilTag detector: sim publisher OR real detector
        apriltag_sim_node,
        apriltag_real_node,

        # MoveIt2 move_group (optional)
        move_group_node,
        rviz_node,

        # Core manipulation nodes
        object_detector_node,
        manipulation_manager_node,

        # OpenCV detection dashboard
        detection_visualizer_node,
    ])
