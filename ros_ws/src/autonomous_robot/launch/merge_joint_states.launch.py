from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Joint state publisher to merge joint states from multiple sources
    # Subscribes to /joint_states (mini PC) and /pi/joint_states (Raspberry Pi)
    # Publishes merged result to /joint_states
    #
    # NOTE: This will cause both the original joint_broad and this merger to publish to /joint_states.
    # The robot_state_publisher will receive both, but since the mini PC joints appear in both messages,
    # you should disable joint_broad on the mini PC to avoid duplicates.
    # Either remove joint_broad_spawner from launch_robot.launch.py or don't launch it.
    joint_state_merger = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': ['/joint_states', '/pi/joint_states'],
            'rate': 50.0,
            'use_sim_time': False
        }],
        output='screen'
    )

    return LaunchDescription([
        joint_state_merger
    ])
