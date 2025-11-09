#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Package directories
    pkg_pan_tilt_description = get_package_share_directory('pan_tilt_description')
    

    # Paths
    
    teleop_config = os.path.join(pkg_pan_tilt_description, 'config', 'pan_tilt_teleop.yaml')

    # Process the URDF
    

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

   

    # ros2 topic pub /pi/pan_tilt_controller/commands std_msgs/msg/Float64MultiArray  "{data: [0.0, 0.0]}"

    

    # Pan Tilt Teleop Node
    pan_tilt_teleop = Node(
        package='pan_tilt_description',
        executable='pan_tilt_teleop',
        name='pan_tilt_teleop',
        output='screen',
        parameters=[teleop_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Joy Node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        
        use_sim_time_arg,
        
        pan_tilt_teleop
    ])
