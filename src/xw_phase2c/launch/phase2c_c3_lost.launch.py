"""Phase2C-C3 LOST recovery — NOT in production robot.launch.py.

Starts reloc_poc (allow_amcl_handoff:=true) + xw_lost_recovery with
phase2c_lost_recovery_enabled:=true.

Also set supervisor phase2c_lost_recovery_enabled:=true (dev only) so heal
arming is suppressed and IDLE cannot mid-cut Reloc ownership.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    maps_dir = LaunchConfiguration('maps_dir')
    map_name = LaunchConfiguration('map_name')

    reloc_launch = os.path.join(
        get_package_share_directory('xw_global_reloc'), 'launch', 'reloc_poc.launch.py'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('maps_dir', default_value='/ros2_ws/maps'),
            DeclareLaunchArgument('map_name', default_value='vp'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(reloc_launch),
                launch_arguments={
                    'allow_amcl_handoff': 'true',
                    'maps_dir': maps_dir,
                    'map_name': map_name,
                }.items(),
            ),
            Node(
                package='xw_phase2c',
                executable='lost_recovery',
                name='xw_lost_recovery',
                parameters=[
                    {
                        'phase2c_lost_recovery_enabled': True,
                        'status2_lost_sec': 6.0,
                        'status3_debounce_sec': 0.5,
                        'p3_max_attempts': 2,
                        'p3_cooldown_sec': 30.0,
                        'auto_resume_nav': True,
                        'auto_resume_follow': False,
                        'auto_resume_recharge': True,
                        'ready_stable_sec': 2.5,
                    }
                ],
                output='screen',
            ),
        ]
    )
