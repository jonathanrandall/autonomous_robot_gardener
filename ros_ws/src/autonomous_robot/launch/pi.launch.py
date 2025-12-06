from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from tempfile import NamedTemporaryFile

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
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("pan_tilt_hardware"), "urdf", "pan_tilt_standalone_pi.xacro"]
                ),
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

#   TF PUBLISHING STRATEGY
#   ----------------------
#   Pi's URDF (pan_tilt_standalone_pi.xacro:55-56):
#     <xacro:pan_tilt_mechanism parent="base_link">
#       <origin xyz="0 0 0.05" rpy="0 0 0"/>
#     Pan_tilt attached to standalone base_link at [0, 0, 0.05] (WRONG for main robot)
#
#   Main robot's URDF (robot_pan_tilt.xacro:9-11):
#     <xacro:pan_tilt_mechanism parent="chassis">
#       <origin xyz="${chassis_length/2-0.08+0.028} -${chassis_width/2-0.088+0.04} ${chassis_height}"
#     rpy="0 0.0 0"/>
#     Pan_tilt attached to chassis at the correct offset (CORRECT)
#
#   THE PROBLEM:
#   Both URDFs define the same link names (pan_tilt_base_link, rgb_camera_optical_link, etc.).
#   Both robot_state_publishers publish TF for these links, causing conflicts. The Pi's transforms
#   (wrong attachment point) sometimes "win", placing the camera at the wrong location.
#
#   THE SOLUTION:
#   Keep robot_state_publisher running (required by ros2_control), but REMAP its TF topics to
#   dummy topics (/pi/tf_unused, /pi/tf_static_unused) that nothing subscribes to. This prevents
#   TF conflicts while keeping robot_state_publisher alive. Only the main robot's
#   robot_state_publisher (from mini_pc.launch.py) publishes to the real /tf and /tf_static topics.
#
#   NOTE: publish_frequency=0.0 doesn't work because robot_state_publisher still publishes in
#   event-driven mode when receiving joint_states. Topic remapping is the only reliable solution.

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="pi",
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/tf', '/pi/tf_unused'),
            ('/tf_static', '/pi/tf_static_unused')
        ]
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

    # Initialize pan/tilt to desired position (pan=1.2, tilt=-0.15)
    init_pan_tilt_position = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once',
             '/pi/pan_tilt_controller/commands',
             'std_msgs/msg/Float64MultiArray',
             '{data: [1.2, -0.15]}'],
        output='screen'
    )

    # Delay the initialization command to run after controller is fully loaded
    delayed_init_pan_tilt = TimerAction(
        period=2.0,  # Wait 2 seconds after controller spawner starts
        actions=[init_pan_tilt_position]
    )

    # Trigger the delayed init after pan_tilt_controller spawner exits
    trigger_init_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pan_tilt_controller_spawner,
            on_exit=[delayed_init_pan_tilt],
        )
    )

    nodes = [
        node_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        delay_pan_tilt_controller_spawner_after_joint_state_broadcaster,
        trigger_init_after_controller,
    ]

    return LaunchDescription(nodes)
