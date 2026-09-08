"""Optional Phase2C-C1 helpers — NOT included in production robot.launch.py.

Starts last_good_pose writer + charger soft-prior helper only.
Supervisor/perception/nav_session changes are already in those packages
(with phase2c_lost_cancel_enabled default false).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    maps_dir = LaunchConfiguration('maps_dir')
    return LaunchDescription(
        [
            DeclareLaunchArgument('maps_dir', default_value='/ros2_ws/maps'),
            Node(
                package='xw_phase2c',
                executable='last_good_pose_writer',
                name='xw_last_good_pose_writer',
                parameters=[
                    {
                        'maps_dir': maps_dir,
                        'tick_hz': 1.0,
                        'stable_sec': 5.0,
                        'block_write_during_follow': True,
                    }
                ],
                output='screen',
            ),
            Node(
                package='xw_phase2c',
                executable='charger_prior_node',
                name='xw_charger_prior',
                parameters=[{'maps_dir': maps_dir, 'publish_hz': 1.0}],
                output='screen',
            ),
        ]
    )
