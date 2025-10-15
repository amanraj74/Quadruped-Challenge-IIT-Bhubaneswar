#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_challenge',
            executable='square_path',
            name='square_path_node',
            output='screen',
            parameters=[],
        ),
    ])

