#!/usr/bin/env python3
import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('xarm_description').find('xarm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'xarm_v3.urdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xarm_v3_ik.urdf')

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
        
        # Launch the position IK GUI
        Node(
            package='xarm_description',
            executable='position_ik_gui.py',
            name='position_ik_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_topic': LaunchConfiguration('trajectory_topic'),
                'urdf_file': urdf_file
            }]
        )
    ])

