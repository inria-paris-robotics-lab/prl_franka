from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    AndSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit


def launch_setup(context):
    # Initialize Arguments
    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    tare_ft_sensor = LaunchConfiguration("tare_ft_sensor")
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
        condition=IfCondition(use_ft_sensor),
    )
    tare_ft_sensor_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/robotiq_ft_hardware_internal_robotiq_ft_sensor/zero_ftsensor",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="log",
        condition=IfCondition(AndSubstitution(tare_ft_sensor, use_ft_sensor)),
    )
    return [
        ft_sensor_controller,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ft_sensor_controller,
                on_exit=tare_ft_sensor_cmd,
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ft_sensor",
            description="Whether to load the force torque sensor controller.",
            default_value="false",
            choices=["true", "false"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tare_ft_sensor",
            description="Whether to tare the force torque sensor on startup.",
            default_value="false",
            choices=["true", "false"],
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
