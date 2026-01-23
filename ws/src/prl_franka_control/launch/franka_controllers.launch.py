from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare


def controller_spawner(controllers, controller_params, active=True):
    inactive_flags = ["--inactive"] if not active else []
    spawners = []
    for controller in controllers:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "10",
                    "--param-file",
                    controller_params,
                ]
                + inactive_flags
                + [controller],
            )
        )
        print("Spawning controllers: ", controller, " Active: ", active)
    return spawners


def launch_setup(context):
    # Arguments
    activate_joint_controller = LaunchConfiguration(
        "activate_joint_controller"
    ).perform(context)
    active_controller = LaunchConfiguration("active_controller").perform(context)
    loaded_controllers = LaunchConfiguration("loaded_controllers").perform(context)
    controller_params = LaunchConfiguration("controller_file").perform(context)

    # Convert string to list
    controllers_to_activate = [c for c in active_controller.split(",") if c]
    controllers_to_load = [c for c in loaded_controllers.split(",") if c]

    # Default active controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "10",
            "joint_state_broadcaster",
            "--param-file",
            controller_params,
        ],
    )
    controllers_active = []
    # Default inactive controllers
    controllers_inactive = []
    # Update active and inactive controllers with initial joint controller
    if activate_joint_controller.lower() == "true":
        for controller in controllers_to_activate:
            controllers_active.append(controller)
        for controller in controllers_to_load:
            if controller not in controllers_active:
                controllers_inactive.append(controller)

    controller_spawners = controller_spawner(
        controllers_active, controller_params
    ) + controller_spawner(controllers_inactive, controller_params, active=False)
    print("Controllers to activate: ", controller_spawners)

    return [
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=controller_spawners,
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate wanted joint controller",
        ),
        DeclareLaunchArgument(
            "active_controller",
            default_value="",
            description="Initially activated robot controller (comma-separated). (used if activate_joint_controller is true)",
        ),
        DeclareLaunchArgument(
            "loaded_controllers",
            default_value="",
            description="Just load the specified controllers (comma-separated).",
        ),
        DeclareLaunchArgument(
            "controller_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_control"),
                    "config",
                    "controllers.yaml",
                ]
            ),
            description="Path to the controller configuration file.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
