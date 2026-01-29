from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context):
    # Initialize Arguments
    force_torque_sensor = LaunchConfiguration("force_torque_sensor")

    # Start the controller based on the argument
    ft_sensor_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
            "--param-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_control"),
                    "config/sensors",
                    "ft300_controller.yaml",
                ]
            ),
            "robotiq_force_torque_sensor_broadcaster",
        ],
        condition=IfCondition(force_torque_sensor),
    )
    return [ft_sensor_controller]  # CORRECTION ICI : On enlève .perform(context)


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "force_torque_sensor",
            description="Whether to load the force torque sensor controller.",
            default_value="false",
            choices=["true", "false"],
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
