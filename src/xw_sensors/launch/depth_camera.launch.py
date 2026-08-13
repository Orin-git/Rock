#!/usr/bin/env python3
"""Launch Angstrong HP60C driver + Gen2 topic bridge (front_up or front_down)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_name = LaunchConfiguration('config').perform(context) or 'depth_camera.yaml'
    cfg_path = os.path.join(
        get_package_share_directory('xw_sensors'),
        'config',
        config_name,
    )

    as_share = get_package_share_directory('ascamera')
    confi_path = os.path.join(as_share, 'configurationfiles')

    import yaml

    with open(cfg_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    fps_arg = (LaunchConfiguration('fps').perform(context) or '').strip()
    preview_arg = (LaunchConfiguration('preview_fps').perform(context) or '').strip()
    points_arg = (LaunchConfiguration('points_fps').perform(context) or '').strip()
    # Empty launch args → use YAML (so config fps/preview_fps take effect).
    fps = int(fps_arg) if fps_arg else int(cfg.get('fps', 5))
    preview_fps = float(preview_arg) if preview_arg else float(cfg.get('preview_fps', 3.0))
    points_fps = float(points_arg) if points_arg else float(cfg.get('points_fps', 10.0))
    enable_pc = LaunchConfiguration('enable_pointcloud').perform(context).lower() in (
        '1', 'true', 'yes', 'on',
    )

    vendor_ns = str(cfg.get('vendor_namespace', 'ascamera_hp60c'))
    bridge_name = str(cfg.get('bridge_node_name', 'xw_depth_topic_bridge'))
    static_name = str(cfg.get('static_tf_node_name', 'camera_front_up_optical_static'))
    robot_frame = str(cfg.get('robot_frame', 'camera_front_up_link'))
    vendor_frame = str(cfg.get('vendor_frame', f'{vendor_ns}_camera_link_0'))
    camera_id = str(cfg.get('camera_id', 'front'))

    ascamera = Node(
        package='ascamera',
        executable='ascamera_node',
        name='camera_publisher',
        namespace=vendor_ns,
        output='screen',
        respawn=True,
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
        name=bridge_name,
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
            'manage_pointcloud_control': bool(cfg.get('manage_pointcloud_control', True)),
            'follow_pointcloud_enabled_topic': bool(
                cfg.get('follow_pointcloud_enabled_topic', False)
            ),
            'gate_rgb_on_sessions': bool(cfg.get('gate_rgb_on_sessions', True)),
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=static_name,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', robot_frame,
            '--child-frame-id', vendor_frame,
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
    )

    return [
        LogInfo(msg=(
            f'[xw_sensors] depth cam id={camera_id} '
            f'usb={cfg.get("usb_bus_no")}/{cfg.get("usb_path")} '
            f'ns={vendor_ns} → {cfg.get("depth_image_out")} '
            f'fps={fps} preview_fps={preview_fps} enable_pointcloud={enable_pc}'
        )),
        ascamera,
        bridge,
        static_tf,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='depth_camera.yaml',
            description='YAML under xw_sensors/config (depth_camera.yaml | depth_camera_front_down.yaml)',
        ),
        # Empty → use values from YAML (do not hardcode 10 here or YAML fps is ignored).
        DeclareLaunchArgument('fps', default_value=''),
        DeclareLaunchArgument('preview_fps', default_value=''),
        DeclareLaunchArgument('points_fps', default_value=''),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='false',
            description='Relay depth/points (CPU heavy; front cam only via set_pointcloud)',
        ),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
