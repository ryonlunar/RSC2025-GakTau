from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_manager",
            executable="map_manager_node",
            name="map_manager",
            output="screen"
        ),
        Node(
            package="path_planner",
            executable="path_planner_node",
            name="path_planner",
            output="screen"
        ),
        Node(
            package="drone_controller",
            executable="drone_controller_node",
            name="drone_controller",
            output="screen"
        )
    ])
