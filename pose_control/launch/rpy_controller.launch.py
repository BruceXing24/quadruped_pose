#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_control',
            executable='rpy_controller',
            name = 'rpy_diagram',
            output = 'screen'

        ),
        Node(
            package='pose_control',
            executable='rpy_controller_1',
            name = 'rpy_pybullet',
            output = 'screen'
        ),
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
            name = 'joystick',
            output = 'screen'
        )
    ]
    )