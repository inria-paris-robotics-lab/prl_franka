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


def launch_setup(context):
    # Initialize Arguments
    gripper = LaunchConfiguration("gripper").perform(context)

    # Start the controller based on the argument
    controllers_to_start = []
    if gripper == "franka_hand":
        gripper_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
                "--param-file",
                f"{FindPackageShare('prl_franka_control').perform(context)}/config/grippers/{gripper}_controller.yaml",
                "gripper_action_controller",
            ],
        )
        controllers_to_start.append(gripper_controller)
    elif gripper in ["ball"]:
        pass
    elif not gripper or gripper.lower() == "none":
        pass
    else:
        raise RuntimeError("Unknown gripper '{}'".format(gripper))
    return controllers_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            description="Gripper type (f: franka_hand, etc .).",
            default_value="",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
