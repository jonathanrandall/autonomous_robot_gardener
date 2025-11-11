#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scale_factor_x',
            default_value='0.4',
            description='Scale factor for pan (X-axis) control'
        ),
        DeclareLaunchArgument(
            'scale_factor_y',
            default_value='0.4',
            description='Scale factor for tilt (Y-axis) control'
        ),
        DeclareLaunchArgument(
            'controller_namespace',
            default_value='pi',
            description='Namespace for the pan_tilt_controller'
        ),

        Node(
            package='pan_tilt_track',
            executable='person_tracker',
            name='person_tracker',
            output='screen',
            parameters=[{
                'scale_factor_x': LaunchConfiguration('scale_factor_x'),
                'scale_factor_y': LaunchConfiguration('scale_factor_y'),
                'controller_namespace': LaunchConfiguration('controller_namespace'),
            }]
        ),
    ])
