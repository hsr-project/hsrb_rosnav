#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
import os

from ament_index_python.packages import get_package_share_directory

from distutils.util import strtobool

from launch import (
    LaunchContext,
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    UnlessCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file to load'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('hsrb_rosnav_config'),
                                       'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_localization', default_value='true',
            description='Whether to use localization'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'odom_topic', default_value='omni_base_controller/wheel_odom',
            description='Odometry topic name'))

    return declared_arguments


def launch_lifecycle_manager_node(context: LaunchContext, args: dict):
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    if not bool(strtobool(context.perform_substitution(args['use_localization']))):
        lifecycle_nodes.append('map_server')

    return [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': args['use_sim_time']},
                {'autostart': args['autostart']},
                {'node_names': lifecycle_nodes}
            ]
        )
    ]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    use_localization = LaunchConfiguration('use_localization')

    args = {}
    for arg in declare_arguments():
        args[arg.name] = LaunchConfiguration(arg.name)

    tf_remappings = [('/tf', 'tf'),
                     ('/tf_static', 'tf_static')]
    velocity_remappings = [('cmd_vel', 'omni_base_controller/cmd_vel')]
    remappings = tf_remappings + velocity_remappings

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local,
        'odom_topic': LaunchConfiguration('odom_topic')}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    env = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),
                                                   'launch',
                                                   'localization_launch.py')),
        launch_arguments={'namespace': namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file}.items(),
        condition=IfCondition(use_localization))

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        respawn=False,
        respawn_delay=2.0,
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml_file}],
        remappings=remappings,
        condition=UnlessCondition(use_localization))

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=tf_remappings)

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=tf_remappings)

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=tf_remappings)

    lifecycle_manager_node = OpaqueFunction(
        function=launch_lifecycle_manager_node,
        args=[args]
    )

    nodes = [
        env,
        localization_launch,
        map_server_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node
    ]

    return LaunchDescription(declare_arguments() + nodes)
