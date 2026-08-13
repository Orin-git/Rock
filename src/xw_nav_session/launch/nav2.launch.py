#!/usr/bin/env python3
"""Nav2 bringup for Gen2: localization + navigation with /xw/cmd/nav remap.

Started by xw_nav_session when entering NAVIGATING (map yaml path as arg).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('xw_nav_session')
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

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
            SetRemap(src='cmd_vel', dst='/xw/cmd/nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': 'False',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]),
    ])
