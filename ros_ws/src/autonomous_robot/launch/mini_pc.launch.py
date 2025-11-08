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
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
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


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='autonomous_robot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/arm/diff_cont/cmd_vel_unstamped')]
        )

    # Override the twist_stamper from joystick.launch.py with correct namespace
    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            remappings=[('/cmd_vel_in','/arm/diff_cont/cmd_vel_unstamped'),
                        ('/cmd_vel_out','/arm/diff_cont/cmd_vel')]
         )

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



    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    namespace = "arm"
    namespaced_yaml = prepend_namespace_to_yaml(namespace, controller_params_file)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm",
        parameters=[robot_description,
                    namespaced_yaml],
        remappings=[('/arm/robot_description', '/robot_description')],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/arm/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/arm/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    hiwonder_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hiwonder_xarm_controller", "--controller-manager", "/arm/controller_manager"],
    )

    delayed_hiwonder_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[hiwonder_controller_spawner],
        )
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_hiwonder_controller_spawner
    ])
