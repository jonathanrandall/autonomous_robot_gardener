#!/usr/bin/env python3

import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from xarm_description.base_robot_gui import BaseRobotGUI

def generate_launch_description():
    pkg_share = FindPackageShare('xarm_description').find('xarm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'xarm_v3.urdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xarm_v3_ik.urdf')
    joystick_config = os.path.join(pkg_share, 'config', 'joystick_ik.yaml')

    # Generate URDF from Xacro
    # subprocess.check_call(['ros2', 'run', 'xacro', 'xacro', xacro_file, '-o', urdf_file])

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
        
        # Launch the joy_node for joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joystick_config, {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Launch the joystick IK GUI
        # Node(
        #     package='xarm_description',
        #     executable='joystick_ik_gui.py',
        #     name='joystick_ik_gui',
        #     output='screen',
        #     parameters=[joystick_config, {
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'trajectory_topic': LaunchConfiguration('trajectory_topic'),
        #         'urdf_file': urdf_file
        #     }]
        # )

        Node(
            package='xarm_description',
            executable='min_joy_listen_v2.py',
            name='min_joy_listen_v2',
            output='screen',
            parameters=[joystick_config, {
                # 'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_topic': LaunchConfiguration('trajectory_topic'),
                'urdf_file': urdf_file
            }]
        )
    ])
