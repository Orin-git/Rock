"""Phase2C-C2 BOOT cascade — NOT in production robot.launch.py.

Starts reloc_poc (allow_amcl_handoff:=true) + boot_localizer + charger_prior.
Ensure Nav2/AMCL/map/scan are up; for isolation set nav_session
phase2c_disable_blind_seed:=true.
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
    auto_start = LaunchConfiguration('auto_start')

    reloc_launch = os.path.join(
        get_package_share_directory('xw_global_reloc'), 'launch', 'reloc_poc.launch.py'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('maps_dir', default_value='/ros2_ws/maps'),
            DeclareLaunchArgument('map_name', default_value='vp'),
            DeclareLaunchArgument('auto_start', default_value='false'),
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
                executable='charger_prior_node',
                name='xw_charger_prior',
                parameters=[{'maps_dir': maps_dir, 'publish_hz': 1.0}],
                output='screen',
            ),
            Node(
                package='xw_phase2c',
                executable='boot_localizer',
                name='xw_boot_localizer',
                parameters=[
                    {
                        'maps_dir': maps_dir,
                        'map_name': map_name,
                        'enabled': True,
                        'auto_start': auto_start,
                        'min_laser_score': 0.38,
                        'sensor_timeout_sec': 90.0,
                        'p3_max_attempts': 2,
                        'p3_cooldown_sec': 30.0,
                    }
                ],
                output='screen',
            ),
        ]
    )
