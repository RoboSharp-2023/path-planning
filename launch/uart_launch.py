from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="path_planning",
            executable="serial_connection",
            name="serial_connection",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"serial_name": "/dev/ttyACM0"}
            ]
        )
    ])
    