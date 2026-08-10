#!/usr/bin/env python3
"""Gen2 system bringup: mock-capable layered stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration('use_sim_hw')
    use_web = LaunchConfiguration('use_web')
    use_foxglove = LaunchConfiguration('use_foxglove')
    profile = LaunchConfiguration('profile')

    desc_share = get_package_share_directory('xw_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'xw_gen2.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    web_share = get_package_share_directory('xw_web')
    web_public = os.path.join(web_share, 'public')

    nodes = [
        LogInfo(msg=['[xw_bringup] profile=', profile, ' use_sim_hw=', use_sim]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='xw_chassis',
            executable='chassis_node',
            name='xw_chassis',
            parameters=[{'use_sim_hw': ParameterValue(use_sim, value_type=bool)}],
            output='screen',
        ),
        Node(
            package='xw_sensors',
            executable='sensors_stub_node',
            name='xw_sensors_stub',
            condition=IfCondition(use_sim),
            output='screen',
        ),
        Node(package='xw_cmd_arbiter', executable='cmd_arbiter_node', name='xw_cmd_arbiter', output='screen'),
        Node(package='xw_safety_gate', executable='safety_gate_node', name='xw_safety_gate', output='screen'),
        Node(package='xw_motion', executable='motion_node', name='xw_motion', output='screen'),
        Node(package='xw_map_manager', executable='map_manager_node', name='xw_map_manager', output='screen'),
        Node(package='xw_slam_session', executable='slam_session_node', name='xw_slam_session', output='screen'),
        Node(package='xw_nav_session', executable='nav_session_node', name='xw_nav_session', output='screen'),
        Node(package='xw_follow_session', executable='follow_session_node', name='xw_follow_session', output='screen'),
        Node(package='xw_fall_session', executable='fall_session_node', name='xw_fall_session', output='screen'),
        Node(package='xw_perception', executable='perception_stub_node', name='xw_perception_stub', output='screen'),
        Node(
            package='xw_supervisor',
            executable='supervisor_node',
            name='xw_supervisor',
            parameters=[{'profile': profile, 'run_mode': 1}],  # profile is string Substitution ok
            output='screen',
        ),
        Node(package='xw_health', executable='topic_health_node', name='xw_topic_health', output='screen'),
        Node(
            package='xw_web',
            executable='web_server',
            name='xw_web',
            parameters=[{'port': 9000, 'web_root': web_public}],
            condition=IfCondition(use_web),
            output='screen',
        ),
    ]

    # Optional foxglove_bridge if installed; do not fail bringup if missing
    foxglove = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then '
            'ros2 run foxglove_bridge foxglove_bridge --ros-args '
            '-p port:=8765 '
            '-p address:=0.0.0.0 '
            '-p topic_whitelist:=["^/tf$","^/tf_static$","^/scan$","^/odom$","^/cmd_vel$",'
            '"^/map$","^/xw/.*","^/safety_status$","^/emergency_stop$"] '
            '-p service_whitelist:=["^/xw/.*"] '
            '-p client_topic_whitelist:=["^/xw/cmd/teleop$","^/xw/goal_pose$","^/initialpose$"] '
            '-p num_threads:=2; '
            'else echo "[xw_bringup] foxglove_bridge not installed, skip WS:8765"; sleep infinity; fi'
        ],
        name='foxglove_bridge_wrap',
        output='screen',
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_hw', default_value='true'),
        DeclareLaunchArgument('use_web', default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('profile', default_value='normal'),
        *nodes,
        foxglove,
    ])
