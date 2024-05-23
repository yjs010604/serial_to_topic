import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_encoder',
            executable='wheel_encoder_node',
            name='wheel_encoder_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB3'},
                {'baud_rate': 115200}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
