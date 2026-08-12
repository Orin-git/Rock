from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    ascamera_node = Node(
        namespace= "ascamera_hp60pro",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "./ascamera/configurationfiles"},
            {"depth_width": 640},
            {"depth_height": 480},
            {"fps": 15},
        ],
        remappings=[]
    )

    # ascamera_node2 = Node(
    #     namespace= "ascamera_hp60pro_2",
    #     package='ascamera',
    #     executable='ascamera_node',
    #     respawn=True,
    #     output='both',
    #     parameters=[
    #         {"usb_bus_no": 3},    # set your usb_bus_no
    #         {"usb_path": "2.2"},  # set your usb_path
    #         {"confiPath": "./ascamera/configurationfiles"},
    #         {"depth_width": 640},
    #         {"depth_height": 480},
    #         {"fps": 15},
    #     ],
    #     remappings=[]
    # )

    ld.add_action(ascamera_node)
    # ld.add_action(ascamera_node2)

    return ld

