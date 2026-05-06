from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() :

    direction_service_node = Node(
        package = "robot_patrol",
        executable = "direction_service_node",
        output = "screen"
    )

    return LaunchDescription([
        direction_service_node
    ])