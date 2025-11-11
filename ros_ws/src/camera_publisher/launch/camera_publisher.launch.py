#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_publisher',
            executable='tof_publisher',
            name='tof_publisher',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='camera_publisher',
            executable='webcam_publisher',
            name='webcam_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ])
