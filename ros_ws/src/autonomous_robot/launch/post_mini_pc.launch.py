from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# You can launch it with:
#   ros2 launch autonomous_robot post_mini_pc.launch.py

#   Or with a custom angle:
#   ros2 launch autonomous_robot post_mini_pc.launch.py angle:=45

def generate_launch_description():
    # Declare launch argument
    angle_arg = DeclareLaunchArgument(
        'angle',
        default_value='0',
        description='Vertical angle parameter for ik_vertical_angle_node'
    )

    # Include merge_joint_states launch file
    merge_joint_states_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('autonomous_robot'),
                'launch',
                'merge_joint_states.launch.py'
            ])
        ])
    )

    # Include pan_tilt.joy launch file
    pan_tilt_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pan_tilt_description'),
                'launch',
                'pan_tilt.joy.launch.py'
            ])
        ])
    )

    image_sync_node = Node(
        package='image_sync_processor',
        executable='image_sync_node',
        name='image_sync_node'
    )

    # camera_to_ee node old version had camera_to_ee.py
    camera_to_ee_node = Node(
        package='autonomous_robot',
        executable='camera_to_ee_pickup.py',
        name='camera_to_ee_pickup_node'
    )

    # ik_vertical_angle_node with parameter
    ik_vertical_angle_node = Node(
        package='xarm_kinematics',
        executable='ik_vertical_angle_node.py',
        name='ik_vertical_angle_node',
        parameters=[{
            'vertical_angle': LaunchConfiguration('angle')
        }]
    )

    ik_arm_pickup_node = Node(
        package='xarm_kinematics',
        executable='ik_arm_pickup.py',
        name='ik_arm_pickup_node',
        parameters=[{
            'vertical_angle': LaunchConfiguration('angle')
        }]
    )

    return LaunchDescription([
        angle_arg,
        merge_joint_states_launch,
        # pan_tilt_joy_launch,
        image_sync_node,
        camera_to_ee_node,
        ik_arm_pickup_node
        # ik_vertical_angle_node
    ])
