from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Custom joint state merger with proper QoS settings
    # Subscribes to /joint_states (mini PC) and /pi/joint_states (Raspberry Pi)
    # Publishes merged result to /merged_joint_states
    #
    # This merger uses TRANSIENT_LOCAL QoS to match joint_state_broadcaster
    # from ros2_control, avoiding QoS incompatibility warnings.
    joint_state_merger = Node(
        package='autonomous_robot',
        executable='joint_state_merger.py',
        name='joint_state_merger',
        output='screen'
    )

    return LaunchDescription([
        joint_state_merger
    ])
