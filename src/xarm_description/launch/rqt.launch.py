from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_joint_trajectory_controller',
            executable='rqt_joint_trajectory_controller',
            name='joint_slider_gui',
            output='screen',
            parameters=[{
                # Pass the robot_description parameter if needed
                'robot_description': '/robot_description',
                # You can override controller names if necessary
                'controller_name': 'hiwonder_xarm_controller'
            }],
            # Remap topics if your controller topics are namespaced differently
            remappings=[
                ('/joint_trajectory_controller/joint_trajectory', '/hiwonder_xarm_controller/joint_trajectory'),
                ('/joint_states', '/joint_states')
            ]
        )
    ])