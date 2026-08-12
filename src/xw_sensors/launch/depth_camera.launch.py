#!/usr/bin/env python3
"""Launch Angstrong HP60C driver + Gen2 topic bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    cfg_path = os.path.join(
        get_package_share_directory('xw_sensors'),
        'config',
        'depth_camera.yaml',
    )

    as_share = get_package_share_directory('ascamera')
    confi_path = os.path.join(as_share, 'configurationfiles')

    import yaml

    with open(cfg_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    fps = int(LaunchConfiguration('fps').perform(context) or cfg.get('fps', 10))
    preview_fps = float(
        LaunchConfiguration('preview_fps').perform(context) or cfg.get('preview_fps', 5.0)
    )
    points_fps = float(
        LaunchConfiguration('points_fps').perform(context) or cfg.get('points_fps', 3.0)
    )
    enable_pc = LaunchConfiguration('enable_pointcloud').perform(context).lower() in (
        '1', 'true', 'yes', 'on',
    )

    ascamera = Node(
        package='ascamera',
        executable='ascamera_node',
        name='camera_publisher',
        namespace='ascamera_hp60c',
        output='screen',
        respawn=False,
        parameters=[{
            'usb_bus_no': int(cfg.get('usb_bus_no', -1)),
            'usb_path': str(cfg.get('usb_path', 'null')),
            'confiPath': confi_path,
            'color_pcl': bool(cfg.get('color_pcl', False)),
            'pub_tfTree': bool(cfg.get('pub_tfTree', True)),
            'depth_width': int(cfg.get('depth_width', 640)),
            'depth_height': int(cfg.get('depth_height', 480)),
            'rgb_width': int(cfg.get('rgb_width', 640)),
            'rgb_height': int(cfg.get('rgb_height', 480)),
            'fps': fps,
        }],
    )

    bridge = Node(
        package='xw_sensors',
        executable='depth_topic_bridge',
        name='xw_depth_topic_bridge',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'rgb_image_in': cfg.get('rgb_image_in'),
            'rgb_info_in': cfg.get('rgb_info_in'),
            'depth_image_in': cfg.get('depth_image_in'),
            'depth_info_in': cfg.get('depth_info_in'),
            'mjpeg_in': cfg.get('mjpeg_in'),
            'points_in': cfg.get('points_in'),
            'rgb_image_out': cfg.get('rgb_image_out'),
            'rgb_info_out': cfg.get('rgb_info_out'),
            'depth_image_out': cfg.get('depth_image_out'),
            'depth_info_out': cfg.get('depth_info_out'),
            'compressed_out': cfg.get('compressed_out'),
            'points_out': cfg.get('points_out'),
            'preview_fps': preview_fps,
            'points_fps': points_fps,
            'relay_raw_rgb': bool(cfg.get('relay_raw_rgb', False)),
            'enable_pointcloud': enable_pc,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_front_optical_static',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'camera_front_link',
            '--child-frame-id', 'ascamera_hp60c_camera_link_0',
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
    )

    return [
        LogInfo(msg=(
            f'[xw_sensors] depth cam confiPath={confi_path} fps={fps} '
            f'preview_fps={preview_fps} enable_pointcloud={enable_pc} points_fps={points_fps}'
        )),
        ascamera,
        bridge,
        static_tf,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('fps', default_value='10'),
        DeclareLaunchArgument('preview_fps', default_value='5.0'),
        DeclareLaunchArgument('points_fps', default_value='3.0'),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='false',
            description='Relay /camera/front/depth/points (CPU heavy; debug only)',
        ),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
