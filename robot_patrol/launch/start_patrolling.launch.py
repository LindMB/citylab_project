from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() :
    
    # Get Package Description and Directory #
    package_description = "robot_patrol"
    package_directory = get_package_share_directory(package_description)

    # Robot Patrol # -> To start robot patrolling
    robot_patrol_node = Node(
        package="robot_patrol",
        executable="robot_patrol_node",
        output="screen",
        emulate_tty=True
    )

    # Load RViz Configuration File #
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(package_directory, "rviz", rviz_config_file)
    print("RViz Config Loaded !")

    # Rviz2 Launch Configuration (RViz)#
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': False}], # Use the real system time
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_patrol_node,
        rviz2_node
    ])
