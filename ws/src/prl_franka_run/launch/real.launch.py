from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
    IfCondition,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
import os
import yaml


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    robot_ip = LaunchConfiguration("robot_ip")
    disable_collision_safety = LaunchConfiguration("disable_collision_safety")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    franka_controllers_setup = LaunchConfiguration("franka_controllers_setup")
    with open(
        os.path.join(franka_controllers_setup.perform(context)), "r"
    ) as setup_file:
        config_controller = yaml.safe_load(setup_file)
    all_controllers = config_controller.get("controllers") or {}
    # YAML may omit lists which yields None; default to empty lists to avoid join errors
    activate_controllers = all_controllers.get("active_controllers") or []
    loaded_controllers = all_controllers.get("inactive_controllers") or []
    disable_collision_safety_bool = (
        context.perform_substitution(disable_collision_safety).lower() == "true"
    )
    default_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("prl_franka_control"),
            "config",
            "default_controllers.yaml",
        ]
    )
    ee_id = LaunchConfiguration("ee_id").perform(context)
    franka_hand = ee_id == "franka_hand"

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            default_controller_params,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        on_exit=Shutdown(),
    )

    ###### Controllers ######
    inactive_controller = ",".join(loaded_controllers)
    active_controllers = ",".join(activate_controllers)
    print("Active controllers: ", active_controllers)
    print("Inactive controllers: ", inactive_controller)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_franka_control"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "active_controller": str(active_controllers),
            "loaded_controllers": str(inactive_controller),
            "controller_file": str(franka_controllers_params.perform(context)),
        }.items(),
    )

    franka_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
        }.items(),
        condition=IfCondition(franka_hand),
    )

    return [
        controller_manager_node,
        franka_gripper_launch,
        controller_launch,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3.",
            choices=["fer", "fr3", "fp3"],
        ),
        DeclareLaunchArgument(
            "disable_collision_safety",
            default_value="false",
            description="Whether to disable safety limits for franka robot.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_control"),
                    "config",
                    "controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "franka_controllers_setup",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_control"),
                    "config",
                    "controller_setup.yaml",
                ]
            ),
            description="Path to the yaml file used to define which controllers to load and activate.",
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
