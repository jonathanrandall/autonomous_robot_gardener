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
    package_name='autonomous_robot'

    # Paths
    urdf_file = os.path.join(pkg_pan_tilt_description, 'urdf', 'pan_tilt_standalone.xacro')
    controller_config = os.path.join(pkg_pan_tilt_description, 'config', 'pan_tilt_controllers.yaml')
    teleop_config = os.path.join(pkg_pan_tilt_description, 'config', 'pan_tilt_teleop.yaml')

    # Process the URDF
    robot_description = Command(['xacro ', urdf_file, ' sim_mode:=true'])

    # Declare launch arguments
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf'
        )   
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
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
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ]),
    #     launch_arguments={
    #         'gz_args': ['-r -v4 ', LaunchConfiguration('world')]
    #     }.items()
    # )

    # ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray \
    # "{data: [0.5, 0.2]}"

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pan_tilt_robot',
            '-z', '0.1',
            '-Y', '3.14159'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Joint State Broadcaster Spawner (Gazebo plugin handles controller_manager)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Pan Tilt Controller Spawner
    pan_tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_tilt_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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
        world_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        pan_tilt_controller_spawner,
        pan_tilt_teleop,
        joy_node,
    ])
