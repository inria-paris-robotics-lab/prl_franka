from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")
    gz_world_path = LaunchConfiguration("gz_world_path")
    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    ee_id = LaunchConfiguration("ee_id")

    gz_verbose_bool = context.perform_substitution(gz_verbose).lower() == "true"
    gz_headless_bool = context.perform_substitution(gz_headless).lower() == "true"
    gz_world_path_str = context.perform_substitution(gz_world_path)
    use_ft_sensor_bool = context.perform_substitution(use_ft_sensor).lower() == "true"
    ee_id_str = context.perform_substitution(ee_id).lower()
    controller_param_file = os.path.join(get_package_share_directory('prl_franka_control'), 'config', 'controllers.yaml')
    config_controller_path = os.path.join(get_package_share_directory('prl_franka_control'), 'config', 'controller_setup.yaml')
    with open(config_controller_path, 'r') as setup_file:
        config_controller = yaml.safe_load(setup_file)
    all_controllers = config_controller.get('controllers') or {}
    # YAML may omit lists which yields None; default to empty lists to avoid join errors
    activate_controllers = all_controllers.get('active_controllers') or []
    loaded_controllers = all_controllers.get('inactive_controllers') or []


    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_world_path_str
            + " -r"
            + f" {'-s' if gz_headless_bool else ''}"
            + f" {'-v 3' if gz_verbose_bool else ''}"
        }.items(),
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "expand_gz_topic_names": True,
                "use_sim_time": True,
                "config_file": PathJoinSubstitution(
                    [
                        FindPackageShare("prl_franka_run"),
                        "config/gazebo",
                        "gz_bridge.yaml",
                    ]
                ),
            }
        ],
        output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ###### Controllers ######
    inactive_controller = ",".join(loaded_controllers)
    active_controllers = ",".join(activate_controllers)
    print("Active controllers: ", active_controllers)
    print("Inactive controllers: ", inactive_controller)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('prl_franka_control'),
            'launch',
            'controllers.launch.py',
            ])
        ]),
        launch_arguments={
            'active_controller': str(active_controllers),
            'loaded_controllers': str(inactive_controller),
            'controller_file':  str(controller_param_file),
        }.items(),
    )


    return [
        gazebo_empty_world,
        ros_gz_bridge_node,
        robot_spawner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawner_node,
                on_exit=[controller_launch],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Whether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Whether to launch Gazebo in headless mode (no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_world_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_description"),
                    "worlds",
                    "empty.sdf",
                ]
            ),
            description="Path to Gazebo world SDF file.",
        ),
        DeclareLaunchArgument(
            "use_ft_sensor",
            default_value="false",
            description="Enable or disable use of force-torque sensor",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="Name of the end effector used.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )