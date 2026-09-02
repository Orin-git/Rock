#!/usr/bin/env python3
"""Gen2 system bringup: mock-capable layered stack.

Real lidar starts after XW_LIDAR_START_DELAY seconds so Web/Foxglove come up first.
RPLidar motor inrush can brown-out the board — only slow-respawn (not a tight loop).
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
    use_gesture = LaunchConfiguration('use_gesture')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_depth_cam = LaunchConfiguration('use_depth_cam')
    use_depth_cam_2 = LaunchConfiguration('use_depth_cam_2')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    use_ekf = LaunchConfiguration('use_ekf')
    use_imu = LaunchConfiguration('use_imu')
    profile = LaunchConfiguration('profile')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_scan_frequency = LaunchConfiguration('lidar_scan_frequency')
    imu_port = LaunchConfiguration('imu_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')

    lidar_delay = float(os.environ.get('XW_LIDAR_START_DELAY', '25'))

    desc_share = get_package_share_directory('xw_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'xw_gen2.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    web_share = get_package_share_directory('xw_web')
    web_public = os.path.join(web_share, 'public')
    # Prefer live source tree for gesture HTTPS so UI edits don't require reinstall.
    _ws = os.environ.get('XW_WS', '/ros2_ws')
    _src_public = os.path.join(_ws, 'src', 'xw_web', 'public')
    gesture_web = _src_public if os.path.isdir(_src_public) else web_public
    gesture_certs = os.path.join(web_share, 'certs', 'gesture')
    _src_certs = os.path.join(_ws, 'src', 'xw_web', 'certs', 'gesture')
    if os.path.isfile(os.path.join(_src_certs, 'cert.pem')):
        gesture_certs = _src_certs
    maps_dir = os.environ.get('XW_MAPS', '/ros2_ws/maps')

    safety_yaml = os.path.join(
        get_package_share_directory('xw_safety_gate'), 'config', 'safety_gate.yaml'
    )

    # When EKF is on: chassis publishes /odom/wheel without TF; ekf publishes /odom + TF.
    chassis_params = {
        'use_sim_hw': ParameterValue(use_sim, value_type=bool),
        'serial_port': LaunchConfiguration('chassis_port'),
        'serial_baud_rate': ParameterValue(
            LaunchConfiguration('chassis_baudrate'), value_type=int
        ),
        'serial_fallback': LaunchConfiguration('chassis_fallback'),
        'publish_odom_tf': ParameterValue(LaunchConfiguration('chassis_publish_odom_tf'), value_type=bool),
        'odom_topic': LaunchConfiguration('chassis_odom_topic'),
        'bms_byte_order': 'big',
        'bms_comm_ok_value': 1,
        'bms_raw_frame_topic': '/bms/raw_frame',
    }

    nodes = [
        LogInfo(msg=[
            '[xw_bringup] profile=', profile,
            ' use_sim_hw=', use_sim,
            ' use_sim_lidar=', use_sim_lidar,
            ' use_depth_cam=', use_depth_cam,
            ' use_depth_cam_2=', use_depth_cam_2,
            ' enable_pointcloud=', enable_pointcloud,
            ' use_ekf=', use_ekf,
            ' use_imu=', use_imu,
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
            parameters=[chassis_params],
            output='screen',
        ),
        Node(
            package='bms_receiver_cpp',
            executable='bms_receiver_node',
            name='bms_receiver_node',
            parameters=[{
                'byte_order': 'big',
                'comm_ok_value': 0,  # this unit: 0 = BMS comm OK
                'raw_frame_topic': '/bms/raw_frame',
                'battery_state_topic': '/battery_state',
                'power_voltage_topic': '/PowerVoltage',
                'charging_flag_topic': '/robot_charging_flag',
                'charging_current_topic': '/robot_charging_current',
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='xw_wt901_imu_cpp',
            executable='wt901_imu_node',
            name='xw_wt901_imu',
            condition=IfCondition(use_imu),
            parameters=[{
                'port': imu_port,
                'port_fallback': '/dev/ttyUSB0',
                'baud': ParameterValue(imu_baudrate, value_type=int),
                'slave_id': 0x50,
                'frame_id': 'imu_link',
                'rate': 15.0,
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='xw_sensors',
            executable='sensors_stub_node',
            name='xw_sensors_stub',
            condition=IfCondition(use_sim_lidar),
            output='screen',
        ),
        Node(package='xw_cmd_arbiter_cpp', executable='cmd_arbiter_node', name='xw_cmd_arbiter', output='screen'),
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
        Node(
            package='xw_sensors',
            executable='pc_nav_filter_node',
            name='xw_pc_nav_filter',
            parameters=[os.path.join(
                get_package_share_directory('xw_sensors'), 'config', 'pc_nav_filter.yaml'
            )],
            condition=IfCondition(use_depth_cam),
            output='screen',
        ),
        Node(
            package='xw_localization_health',
            executable='localization_health_node',
            name='xw_localization_health',
            output='screen',
        ),
        Node(package='xw_motion_cpp', executable='motion_node', name='xw_motion', output='screen'),
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
        Node(
            package='xw_nav_session',
            executable='nav_session_node',
            name='xw_nav_session',
            parameters=[{'maps_dir': maps_dir, 'use_nav2': True}],
            output='screen',
        ),
        Node(package='xw_follow_session', executable='follow_session_node', name='xw_follow_session', output='screen'),
        Node(
            package='xw_recharge',
            executable='recharge_node',
            name='xw_recharge',
            parameters=[os.path.join(
                get_package_share_directory('xw_recharge'), 'config', 'recharge.yaml'
            )],
            output='screen',
        ),
        Node(
            package='xw_explore',
            executable='explore_session_node',
            name='xw_explore_session',
            parameters=[{'maps_dir': maps_dir}],
            output='screen',
        ),
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
            respawn=True,
            respawn_delay=2.0,
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
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depth_share, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_ekf': use_ekf}.items(),
    )
    depth_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_launch),
        launch_arguments={
            'config': 'depth_camera.yaml',
            'enable_pointcloud': enable_pointcloud,
        }.items(),
        condition=IfCondition(use_depth_cam),
    )
    # Stagger cam2 so dual HP60C USB init does not race / brown-out the bus.
    depth_cam_2 = TimerAction(
        period=8.0,
        actions=[
            LogInfo(
                msg='[xw_bringup] starting depth camera #2 (front_down) after 8s',
                condition=IfCondition(use_depth_cam_2),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depth_launch),
                launch_arguments={
                    'config': 'depth_camera_front_down.yaml',
                    'enable_pointcloud': 'false',
                }.items(),
                condition=IfCondition(use_depth_cam_2),
            ),
        ],
    )

    foxglove = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then '
            'ros2 run foxglove_bridge foxglove_bridge --ros-args '
            '-p port:=8765 '
            '-p address:=0.0.0.0 '
            '-p topic_whitelist:=["^/tf$","^/tf_static$","^/scan$","^/odom$","^/cmd_vel$",'
            '"^/imu/data$","^/map$","^/plan$","^/local_plan$","^/xw/.*","^/safety_status$",'
            '"^/camera/front_up/color/image_raw/compressed$","^/camera/front_up/.*/camera_info$",'
            '"^/camera/front_up/depth/points$",'
            '"^/camera/front_down/color/image_raw/compressed$","^/camera/front_down/.*/camera_info$",'
            '"^/camera/front_down/depth/points$"] '
            '-p service_whitelist:=["^/xw/.*"] '
            '-p client_topic_whitelist:=["^/xw/cmd/teleop$","^/xw/goal_pose$","^/initialpose$"] '
            '-p num_threads:=2; '
            'else echo "[xw_bringup] foxglove_bridge not installed, skip WS:8765"; sleep infinity; fi'
        ],
        name='foxglove_bridge_wrap',
        output='screen',
        condition=IfCondition(use_foxglove),
    )

    gesture_https = Node(
        package='xw_web',
        executable='gesture_https',
        name='xw_gesture_https',
        arguments=[
            '--web-dir', gesture_web,
            '--port', '9443',
            '--cert-dir', gesture_certs,
            '--serve',
        ],
        condition=IfCondition(use_gesture),
        output='screen',
        respawn=False,
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
                parameters=[os.path.join(
                    get_package_share_directory('xw_bringup'), 'config', 'lidar_params.yaml'
                )],
                output='screen',
                # Soft-fail UART (no assert). Slow respawn after power/signal-split recovers.
                respawn=True,
                respawn_delay=30.0,
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
                              description='Start front HP60C #1 → /camera/front_up/...'),
        DeclareLaunchArgument('use_depth_cam_2', default_value='true',
                              description='Start front HP60C #2 → /camera/front_down/...'),
        DeclareLaunchArgument('enable_pointcloud', default_value='false',
                              description='Relay /camera/front_up/depth/points for Foxglove debug (CPU heavy)'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Fuse /odom/wheel + /imu/data via robot_localization (needs independent IMU)',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Start WT901C485 Modbus driver → /imu/data (frame imu_link)',
        ),
        DeclareLaunchArgument(
            'chassis_odom_topic',
            default_value='odom/wheel',
            description='Wheel odom topic; use odom only when use_ekf:=false',
        ),
        DeclareLaunchArgument(
            'chassis_publish_odom_tf',
            default_value='false',
            description='false when EKF owns odom→base_link; true only if use_ekf:=false',
        ),
        DeclareLaunchArgument('lidar_port', default_value='/dev/radar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='1000000'),
        DeclareLaunchArgument(
            'lidar_scan_frequency',
            default_value='10.0',  # S3 native 600RPM; 20Hz(1200RPM) hangs setMotorSpeed
            description='RPLidar motor scan rate (Hz)',
        ),
        DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
        DeclareLaunchArgument('imu_baudrate', default_value='9600'),
        DeclareLaunchArgument('use_web', default_value='true'),
        DeclareLaunchArgument(
            'use_gesture',
            default_value='false',
            description='Always-on HTTPS:9443 gesture teleop (debug). Default off; web /api/gesture starts it on demand.',
        ),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('profile', default_value='normal'),
        *nodes,
        depth_cam,
        depth_cam_2,
        ekf_launch,
        foxglove,
        gesture_https,
        delayed_lidar,
    ])
