from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_sync_processor',
            executable='image_sync_node',
            name='image_sync_processor',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])
