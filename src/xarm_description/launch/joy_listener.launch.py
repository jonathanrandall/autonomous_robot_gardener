#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """Launch the joy listener node."""
    
    # Get the path to the script - use absolute path from workspace root
    script_path = os.path.join(
        '/workspace/project_ws/src/xarm_description/scripts/min_joy_listen.py'
    )
    
    joy_listener_node = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen',
        name='joy_listener'
    )
    
    return LaunchDescription([
        joy_listener_node
    ])
