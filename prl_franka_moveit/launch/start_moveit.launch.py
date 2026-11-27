import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    ############################ MoveIt Part ################################

    # Config file
    moveit_config = MoveItConfigsBuilder(
        robot_name="prl_franka", package_name="prl_franka_moveit"
    ).to_moveit_configs()
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("prl_franka_moveit"), "config", "moveit.rviz"]
    )

    # Launch MoveIt
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": "true",
                # clamp start states a hair outside the limits (sim/encoder noise) instead of rejecting them
                **{
                    f"{p}.fix_start_state": True
                    for p in moveit_config.planning_pipelines["planning_pipelines"]
                },
            },
        ],
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    wait_for_joint_states = Node(
        package="prl_franka_moveit",
        executable="wait_for_joint_states",
        output="screen",
    )

    return [
        wait_for_joint_states,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_joint_states,
                on_exit=[
                    node_move_group,
                    rviz_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="true",
            description="Use simulation time if true, wall clock time otherwise.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
