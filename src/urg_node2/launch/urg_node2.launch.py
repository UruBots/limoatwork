# Copyright 2022 eSOL Co.,Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node

def generate_launch_description():

    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_solver.yaml'
    )

    # パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # urg_node2をライフサイクルノードとして起動
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )   

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0.12', '0', '0','0', '0', '0', 'base_link','laser_frame'],
                    condition=IfCondition(LaunchConfiguration('publish_tf'))
                    )


    #tf2_node = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_tf_pub_laser',   # antes era node_name
    #    arguments=[
    #        '0', '0', '0.02',  # x y z
    #        '0', '0', '0',     # roll pitch yaw
    #        'base_link',       # frame pai
    #        'laser_frame'      # frame filho
    #    ]
    #)

    # パラメータについて
    # auto_start      : 起動時自動でActive状態まで遷移 (default)true
    # node_name       : ノード名 (default)"urg_node2"
    # scan_topic_name : トピック名 (default)"scan" *マルチエコー非対応*
        # Publicador estático para base_link -> base_scan 
    
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='scan'),
        publish_tf_arg,
        lifecycle_node,
        urg_node2_node_configure_event_handler,
        urg_node2_node_activate_event_handler,
        tf2_node,
      
    ])

