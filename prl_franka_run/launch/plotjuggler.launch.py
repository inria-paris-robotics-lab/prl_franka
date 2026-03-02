import ast
from copy import deepcopy

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    OrSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
import yaml
from pathlib import Path


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    plotjuggler_config_file = LaunchConfiguration("plotjuggler_config_file")

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--layout", plotjuggler_config_file],
    )

    return [
        plotjuggler_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "plotjuggler_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_description"),
                    "plotjuggler",
                    "config.xml",
                ]
            ),
            description="Path to PlotJuggler configuration file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
