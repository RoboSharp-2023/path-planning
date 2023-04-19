from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="path_planning",
                executable="path_visualizer.py",
                name="path_visualizer",
                output='screen'
            ),
        
            Node(
                package="path_planning",
                executable="path_server",
                name="path_planer",
                output='screen'
            ),
        ]
    )