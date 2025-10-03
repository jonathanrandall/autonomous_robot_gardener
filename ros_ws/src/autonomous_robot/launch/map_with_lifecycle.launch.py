from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/workspaces/jazzy_docker/autonomous_robot_gardener/ros_ws/my_map_save.yaml',
                'use_sim_time': True
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': True
            }]
        ),

        # Static transform publisher (map -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            arguments=['0','0','0','0','0','0','map','odom']
        )
    ])
