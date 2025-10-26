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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    urdf_file = os.path.join(pkg_pan_tilt_description, 'urdf', 'pan_tilt_standalone.xacro')
    controller_config = os.path.join(pkg_pan_tilt_description, 'config', 'pan_tilt_controllers.yaml')
    teleop_config = os.path.join(pkg_pan_tilt_description, 'config', 'pan_tilt_teleop.yaml')

    # Process the URDF
    robot_description = Command(['xacro ', urdf_file, ' sim_mode:=true'])

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='World file to load in Gazebo'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', LaunchConfiguration('world')]
        }.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pan_tilt_robot',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Pan Tilt Controller Spawner
    pan_tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_tilt_controller'],
        output='screen'
    )

    # Pan Tilt Teleop Node
    pan_tilt_teleop = Node(
        package='pan_tilt_description',
        executable='pan_tilt_teleop',
        name='pan_tilt_teleop',
        output='screen',
        parameters=[teleop_config]
    )

    # Joy Node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        pan_tilt_controller_spawner,
        pan_tilt_teleop,
        joy_node,
    ])
