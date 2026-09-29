#!/usr/bin/env python3
"""Nav2 bringup for Gen2: localization + navigation + collision_monitor.

Controller → velocity_smoother(cmd_vel_smoothed) → collision_monitor → /xw/cmd/nav.
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
    share = get_package_share_directory('xw_nav_session')
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    bt_xml = os.path.join(share, 'behavior_trees', 'navigate_to_pose_gen2.xml')
    cm_params = os.path.join(share, 'config', 'collision_monitor.yaml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_xml,
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
            default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Absolute path to map.yaml',
        ),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # xw fork（见该文件抬头）：唯一改动 = 给 LM 补 bond_timeout=10.0。
                    # 用自家 share 而非原厂 bringup_dir —— 原厂文件被 apt upgrade 覆盖会冲掉补丁。
                    os.path.join(share, 'launch', 'localization_gen2_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': 'False',
                }.items(),
            ),
            # Gen2 fork: do NOT remap cmd_vel_smoothed → cmd_vel (conflicts with safety_gate).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, 'launch', 'navigation_gen2_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items(),
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                # Dedicated YAML — full multi-node RewrittenYaml drops nested CM overrides.
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
                    # xw: 与 localization 同因——4s 门限在 Rock 5T 上过紧（DDS 投递静默会超 4s）。
                    # 它死了 => /xw/cmd/nav 静默 => 没有东西在命令车动 => 反是 fail-safe。
                    'bond_timeout': 10.0,
                }],
            ),
        ]),
    ])
