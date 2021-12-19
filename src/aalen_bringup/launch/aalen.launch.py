from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    gnss_node = Node(
        package="aalen",
        executable="gnss",
        name="gnss_node",
    )

    ld.add_action(gnss_node)
    
    return ld