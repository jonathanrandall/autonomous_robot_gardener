#!/usr/bin/env python3

# Sometimes the port isn’t “busy” but the ESP32 is stuck. 
# You can toggle dtr/rts or just replug, but in software you can also reset it:
# stty -F /dev/ttyUSB0 hupcl


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('xarm_hardware')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    # set use_sim_time to false
    use_sim_time = 'false'
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control = 'true'
    serial_port_esp32 = LaunchConfiguration('serial_port_esp32')

    # Process the URDF file
    robot_dir = os.path.join(get_package_share_directory('xarm_description'))
    xacro_file = os.path.join(robot_dir,'urdf','xarm_world.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, 
                ' sim_mode:=', use_sim_time,' serial_port_esp32:=', serial_port_esp32])
    clear_usb = ExecuteProcess(
        cmd=['fuser', '-k', serial_port_esp32],
        output='screen'
    )
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config}#, 'use_sim_time': use_sim_time}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    robot_controllers = os.path.join(robot_dir, 'config', 'xarm_controllers.yaml')
    rviz_config_file = os.path.join(robot_dir, "config",  "xarm.rviz")


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='controller_manager',
    #     output='screen',
    #     parameters=[robot_controllers]
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers, "--ros-args", "--log-level", "info"],
        output="both",
    )
    delayed_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=clear_usb,
            on_exit=[control_node]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hiwonder_xarm_controller", "--controller-manager", "/controller_manager"],
    )
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    
    # Launch arguments
    # serial_port = LaunchConfiguration('serial_port')
    # baud_rate = LaunchConfiguration('baud_rate')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ROS2 control'
        ),
        DeclareLaunchArgument(
            'serial_port_esp32',
            default_value='/dev/esp32_arm',
            description='Port for ESP32'
        ),
        # print port_for
        # DeclareLaunchArgument(
        #     'port_for_esp32',
        #     default_value='/dev/ttyUSB0',
        #     description='Port for ESP32'
        # ),
        clear_usb,
        # control_node,
        delayed_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
