#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/radar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    # Must match URDF / static TF (base_link -> base_scan) on brushless_senior_diff.
    frame_id = LaunchConfiguration('frame_id', default='base_scan')
    topic_name = LaunchConfiguration('topic_name', default='scan')
    inverted = LaunchConfiguration('inverted', default='false')
    flip_x_axis = LaunchConfiguration('flip_x_axis', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    enable_filter = LaunchConfiguration('enable_filter', default='true')
    filter_regions = LaunchConfiguration(
        'filter_regions', default='[-120.0, -60.0, 60.0, 120.0]')
    # filter_inclusive=false: drop points inside the regions (robot body blind zones).
    filter_inclusive = LaunchConfiguration('filter_inclusive', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'topic_name',
            default_value=topic_name,
            description='Specifying topic_name of lidar scan'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'flip_x_axis',
            default_value=flip_x_axis,
            description='Specifying whether or not to flip scan data on x-axis'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        DeclareLaunchArgument(
            'enable_filter',
            default_value=enable_filter,
            description='Enable angle filtering for lidar data'),

        DeclareLaunchArgument(
            'filter_regions',
            default_value=filter_regions,
            description='Filter regions as [min1, max1, min2, max2, ...] in degrees'),

        DeclareLaunchArgument(
            'filter_inclusive',
            default_value=filter_inclusive,
            description='If true, keep points within the angle ranges; if false, filter them out'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'topic_name': topic_name,
                'inverted': inverted,
                'flip_x_axis': flip_x_axis,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'enable_filter': enable_filter,
                'filter_regions': filter_regions,
                'filter_inclusive': filter_inclusive,
            }],
            output='screen',
            # Safety net if the process exits hard; in-node reconnect covers soft USB stalls.
            respawn=True,
            respawn_delay=2.0),
    ])
