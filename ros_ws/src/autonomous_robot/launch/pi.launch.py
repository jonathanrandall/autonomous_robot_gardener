from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_directory

def prepend_namespace_to_yaml(namespace, yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    namespaced = {namespace: data}
    tmp = NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(namespaced, tmp)
    tmp.close()
    return tmp.name


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("pan_tilt_hardware"), "urdf", "pan_tilt_standalone_pi.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="pi",
        output='screen',
        parameters=[robot_description]
    )


    controller_params_file = os.path.join(get_package_share_directory("pan_tilt_hardware"),'config','pan_tilt_controllers.yaml')

    namespace = "pi"
    namespaced_yaml = prepend_namespace_to_yaml(namespace, controller_params_file)

    # Controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="pi",
        parameters=[robot_description, namespaced_yaml],
        # remappings=[('/pi/robot_description', '/robot_description')],
        output="screen",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/pi/controller_manager"],
    )

    # Pan-tilt controller spawner
    pan_tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_controller", "--controller-manager", "/pi/controller_manager"],
    )

    # Delay pan_tilt_controller start after joint_state_broadcaster
    delay_pan_tilt_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[pan_tilt_controller_spawner],
        )
    )

    nodes = [
        node_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        delay_pan_tilt_controller_spawner_after_joint_state_broadcaster,
    ]

    return LaunchDescription(nodes)
