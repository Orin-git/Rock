#!/usr/bin/env python3
"""Explore Nav2: navigation stack only (no AMCL / map_server).

SLAM Toolbox owns /map and map→odom; static_layer follows live /map.
Collision monitor → /xw/cmd/nav (same as normal nav).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    explore_share = get_package_share_directory('xw_explore')
    nav_share = get_package_share_directory('xw_nav_session')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml = LaunchConfiguration('default_bt_xml')

    cm_params = os.path.join(nav_share, 'config', 'collision_monitor.yaml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': default_bt_xml,
        },
        convert_types=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(explore_share, 'config', 'nav2_params_explore.yaml'),
        ),
        DeclareLaunchArgument(
            'default_bt_xml',
            default_value=os.path.join(
                explore_share, 'behavior_trees', 'navigate_to_pose_explore.xml'
            ),
        ),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'navigation_gen2_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': 'False',
                    'container_name': 'explore_nav2_container',
                }.items(),
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[cm_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_collision_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['collision_monitor'],
                    'bond_timeout': 4.0,
                }],
            ),
        ]),
    ])
