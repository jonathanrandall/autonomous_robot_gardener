#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'trajectory_topic',
            default_value='/hiwonder_xarm_controller/joint_trajectory',
            description='Topic for publishing joint trajectories'
        ),
        
        # Launch the joint slider GUI
        Node(
            package='xarm_description',
            executable='joint_slider_gui.py',
            name='joint_slider_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_topic': LaunchConfiguration('trajectory_topic')
            }]
        )
    ])
