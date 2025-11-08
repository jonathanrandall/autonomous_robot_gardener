import os
import yaml
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def prepend_namespace_to_yaml(namespace, yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    namespaced = {namespace: data}
    tmp = NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(namespaced, tmp)
    tmp.close()
    return tmp.name


def generate_launch_description():

    package_name='autonomous_robot'

    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "description",
        "robot.urdf.xacro"
    ])

    robot_description = {
        "robot_description": Command([
            "xacro", " ", xacro_file,
            " use_ros2_control:=true",
            " sim_mode:=false"
        ])
    }

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    namespace = "pi"
    namespaced_yaml = prepend_namespace_to_yaml(namespace, controller_params_file)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="pi",
        parameters=[robot_description,
                    namespaced_yaml],
        remappings=[('/pi/robot_description', '/robot_description')],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Only spawn pan_tilt controller and joint state broadcaster for the Pi
    pan_tilt_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_controller", "--controller-manager", "/pi/controller_manager"],
    )

    delayed_pan_tilt_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[pan_tilt_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/pi/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        delayed_controller_manager,
        delayed_pan_tilt_spawner,
        delayed_joint_broad_spawner
    ])
