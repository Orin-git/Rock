from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    """Dev-only Relocalizer launch.

    Production robot.launch.py does NOT include this node.
    Set allow_amcl_handoff:=true only for Phase2B AMCL handoff validation.
    """
    share = get_package_share_directory('xw_global_reloc')
    params = os.path.join(share, 'config', 'reloc_poc.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('map_name', default_value='vp'),
            DeclareLaunchArgument('maps_dir', default_value='/ros2_ws/maps'),
            DeclareLaunchArgument('db_root', default_value=''),
            # Default false keeps accidental launches dry-run-safe.
            DeclareLaunchArgument('allow_amcl_handoff', default_value='false'),
            Node(
                package='xw_global_reloc',
                executable='global_reloc_poc',
                name='xw_global_reloc_poc',
                output='screen',
                parameters=[
                    params,
                    {
                        'map_name': LaunchConfiguration('map_name'),
                        'maps_dir': LaunchConfiguration('maps_dir'),
                        'db_root': LaunchConfiguration('db_root'),
                        'allow_amcl_handoff': LaunchConfiguration('allow_amcl_handoff'),
                    },
                ],
            ),
        ]
    )
