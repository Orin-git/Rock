#!/usr/bin/env python3
"""Gen2 system bringup: mock-capable layered stack.

Real lidar starts after XW_LIDAR_START_DELAY seconds so Web/Foxglove come up first.
RPLidar motor inrush can brown-out the board — never respawn it in a tight loop.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration('use_sim_hw')
    use_sim_lidar = LaunchConfiguration('use_sim_lidar')
    use_web = LaunchConfiguration('use_web')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_depth_cam = LaunchConfiguration('use_depth_cam')
    use_depth_cam_2 = LaunchConfiguration('use_depth_cam_2')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    profile = LaunchConfiguration('profile')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')

    lidar_delay = float(os.environ.get('XW_LIDAR_START_DELAY', '25'))

    desc_share = get_package_share_directory('xw_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'xw_gen2.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    web_share = get_package_share_directory('xw_web')
    web_public = os.path.join(web_share, 'public')
    maps_dir = os.environ.get('XW_MAPS', '/ros2_ws/maps')

    safety_yaml = os.path.join(
        get_package_share_directory('xw_safety_gate'), 'config', 'safety_gate.yaml'
    )

    nodes = [
        LogInfo(msg=[
            '[xw_bringup] profile=', profile,
            ' use_sim_hw=', use_sim,
            ' use_sim_lidar=', use_sim_lidar,
            ' use_depth_cam=', use_depth_cam,
            ' use_depth_cam_2=', use_depth_cam_2,
            ' enable_pointcloud=', enable_pointcloud,
            f' lidar_delay={lidar_delay}s',
        ]),
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
            parameters=[{
                'use_sim_hw': ParameterValue(use_sim, value_type=bool),
                'serial_port': LaunchConfiguration('chassis_port'),
                'serial_baud_rate': ParameterValue(
                    LaunchConfiguration('chassis_baudrate'), value_type=int
                ),
                'serial_fallback': LaunchConfiguration('chassis_fallback'),
            }],
            output='screen',
        ),
        Node(
            package='xw_sensors',
            executable='sensors_stub_node',
            name='xw_sensors_stub',
            condition=IfCondition(use_sim_lidar),
            output='screen',
        ),
        Node(package='xw_cmd_arbiter', executable='cmd_arbiter_node', name='xw_cmd_arbiter', output='screen'),
        Node(
            package='xw_safety_gate',
            executable='safety_gate_node',
            name='xw_safety_gate',
            parameters=[
                safety_yaml,
                {'use_depth': ParameterValue(use_depth_cam, value_type=bool)},
            ],
            output='screen',
        ),
        Node(package='xw_motion', executable='motion_node', name='xw_motion', output='screen'),
        Node(
            package='xw_map_manager',
            executable='map_manager_node',
            name='xw_map_manager',
            parameters=[{'maps_dir': maps_dir}],
            output='screen',
        ),
        Node(
            package='xw_slam_session',
            executable='slam_session_node',
            name='xw_slam_session',
            parameters=[{
                'maps_dir': maps_dir,
                'base_frame': 'base_link',
                'map_frame': 'map',
            }],
            output='screen',
        ),
        Node(package='xw_nav_session', executable='nav_session_node', name='xw_nav_session', output='screen'),
        Node(package='xw_follow_session', executable='follow_session_node', name='xw_follow_session', output='screen'),
        Node(package='xw_fall_session', executable='fall_session_node', name='xw_fall_session', output='screen'),
        Node(
            package='xw_perception',
            executable='person_perception_node',
            name='xw_perception',
            output='screen',
        ),
        Node(
            package='xw_supervisor',
            executable='supervisor_node',
            name='xw_supervisor',
            parameters=[{'profile': profile, 'run_mode': 1}],
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
            respawn=True,
            respawn_delay=3.0,
        ),
    ]

    depth_share = get_package_share_directory('xw_sensors')
    depth_launch = os.path.join(depth_share, 'launch', 'depth_camera.launch.py')
    depth_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_launch),
        launch_arguments={
            'config': 'depth_camera.yaml',
            'enable_pointcloud': enable_pointcloud,
        }.items(),
        condition=IfCondition(use_depth_cam),
    )
    depth_cam_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_launch),
        launch_arguments={
            'config': 'depth_camera_front_2.yaml',
            'enable_pointcloud': 'false',
        }.items(),
        condition=IfCondition(use_depth_cam_2),
    )

    foxglove = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then '
            'ros2 run foxglove_bridge foxglove_bridge --ros-args '
            '-p port:=8765 '
            '-p address:=0.0.0.0 '
            '-p topic_whitelist:=["^/tf$","^/tf_static$","^/scan$","^/odom$","^/cmd_vel$",'
            '"^/map$","^/xw/.*","^/safety_status$",'
            '"^/camera/front/color/image_raw/compressed$","^/camera/front/.*/camera_info$",'
            '"^/camera/front/depth/points$",'
            '"^/camera/front_2/color/image_raw/compressed$","^/camera/front_2/.*/camera_info$",'
            '"^/camera/front_2/depth/points$"] '
            '-p service_whitelist:=["^/xw/.*"] '
            '-p client_topic_whitelist:=["^/xw/cmd/teleop$","^/xw/goal_pose$","^/initialpose$"] '
            '-p num_threads:=2; '
            'else echo "[xw_bringup] foxglove_bridge not installed, skip WS:8765"; sleep infinity; fi'
        ],
        name='foxglove_bridge_wrap',
        output='screen',
        condition=IfCondition(use_foxglove),
    )

    delayed_lidar = TimerAction(
        period=lidar_delay,
        actions=[
            LogInfo(msg=[f'[xw_bringup] starting real rplidar after {lidar_delay:.0f}s delay']),
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                condition=UnlessCondition(use_sim_lidar),
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': lidar_port,
                    'serial_baudrate': ParameterValue(lidar_baudrate, value_type=int),
                    'frame_id': 'lidar_link',
                    'topic_name': 'scan',
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
                    'enable_filter': False,
                }],
                output='screen',
                respawn=False,
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_hw', default_value='true',
                              description='Simulate chassis odometry (independent of lidar)'),
        DeclareLaunchArgument('chassis_port', default_value='/dev/chassis',
                              description='STM32 chassis serial device'),
        DeclareLaunchArgument('chassis_baudrate', default_value='115200'),
        DeclareLaunchArgument('chassis_fallback', default_value='/dev/ttyACM0',
                              description='Fallback if /dev/chassis missing'),
        DeclareLaunchArgument('use_sim_lidar', default_value='false',
                              description='If true, stub /scan; if false, delayed rplidar_ros'),
        DeclareLaunchArgument('use_depth_cam', default_value='true',
                              description='Start front HP60C #1 → /camera/front/...'),
        DeclareLaunchArgument('use_depth_cam_2', default_value='true',
                              description='Start front HP60C #2 → /camera/front_2/...'),
        DeclareLaunchArgument('enable_pointcloud', default_value='false',
                              description='Relay /camera/front/depth/points for Foxglove debug (CPU heavy)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/radar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='1000000'),
        DeclareLaunchArgument('use_web', default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('profile', default_value='normal'),
        *nodes,
        depth_cam,
        depth_cam_2,
        foxglove,
        delayed_lidar,
    ])
