#!/usr/bin/env python3
"""EKF odom fusion slot: wheel (/odom/wheel) + independent IMU (/imu/data) → /odom.

Enable with use_ekf:=true. Requires robot_localization and a live /imu/data publisher.
Chassis must publish odom_topic:=odom/wheel and publish_odom_tf:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_yaml = os.path.join(
        get_package_share_directory('xw_sensors'), 'config', 'ekf.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Start ekf_node (needs /imu/data + chassis /odom/wheel)',
        ),
        LogInfo(
            msg=['[xw_sensors] EKF slot use_ekf=', use_ekf, ' yaml=', ekf_yaml],
            condition=IfCondition(use_ekf),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml],
            remappings=[('odometry/filtered', 'odom')],
            condition=IfCondition(use_ekf),
        ),
    ])
