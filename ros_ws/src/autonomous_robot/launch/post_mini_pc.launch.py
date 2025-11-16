from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    # camera_to_ee node
    camera_to_ee_node = Node(
        package='autonomous_robot',
        executable='camera_to_ee.py',
        name='camera_to_ee'
    )

    # ik_vertical_angle_node with parameter
    ik_vertical_angle_node = Node(
        package='xarm_description',
        executable='ik_vertical_angle_node.py',
        name='ik_vertical_angle_node',
        parameters=[{
            'vertical_angle': LaunchConfiguration('angle')
        }]
    )

    return LaunchDescription([
        angle_arg,
        merge_joint_states_launch,
        pan_tilt_joy_launch,
        camera_to_ee_node,
        ik_vertical_angle_node
    ])
