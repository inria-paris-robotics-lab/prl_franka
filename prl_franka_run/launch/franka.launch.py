import ast
import os
import tempfile
import re
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
    config_file = LaunchConfiguration("robot_config_file").perform(context)
    config_path = Path(config_file)
    with config_path.open("r") as setup_file:
        robot = yaml.safe_load(setup_file)
        config = robot.get("ROBOT", {})
    arm_id = config.get("arm_id", "fr3v2")
    if arm_id not in ["fer", "fr3", "fp3", "fr3v2"]:
        raise RuntimeError(
            f"Unsupported arm_id '{arm_id}' in robot configuration file! "
            "Supported values are: 'fer', 'fr3', 'fr3v2', 'fp3'."
        )
    namespaces = config.get("namespaces", "")
    ee_id = config.get("ee_id", "franka_hand")
    load_end_effector = str(
        config.get("load_end_effector", True)
    ).lower()  # "true"/"false" for launch args
    robot_ip = config.get("robot_ip", "")
    arm_id = config.get("arm_id", "fr3v2")
    use_ft_sensor = str(config.get("use_ft_sensor", False)).lower()
    tare_ft_sensor = str(config.get("tare_ft_sensor", False)).lower()

    aux_computer_ip = LaunchConfiguration("aux_computer_ip")
    aux_computer_user = LaunchConfiguration("aux_computer_user")
    on_aux_computer = LaunchConfiguration("on_aux_computer")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    plotjuggler_config_file = LaunchConfiguration("plotjuggler_config_file")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_mujoco = LaunchConfiguration("use_mujoco")
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")
    gz_world_path = LaunchConfiguration("gz_world_path")

    external_controllers_params = LaunchConfiguration("external_controllers_params")
    external_controllers_names = LaunchConfiguration("external_controllers_names")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    franka_controllers_setup = LaunchConfiguration("franka_controllers_setup")
    initial_joint_position = LaunchConfiguration("initial_joint_position")
    use_plotjuggler = LaunchConfiguration("use_plotjuggler")
    activate_cameras = LaunchConfiguration("activate_cameras")
    use_moveit = LaunchConfiguration("use_moveit")

    robot_ip_empty = robot_ip == ""
    aux_computer_ip_empty = context.perform_substitution(aux_computer_ip) == ""
    aux_computer_user_empty = context.perform_substitution(aux_computer_user) == ""
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    use_mujoco_bool = context.perform_substitution(use_mujoco).lower() == "true"
    if use_gazebo_bool and use_mujoco_bool:
        raise RuntimeError(
            "Choose one simulator: `use_gazebo:=true` or `use_mujoco:=true`."
        )
    use_sim_time = use_gazebo_bool or use_mujoco_bool
    mujoco_dir = os.path.join(tempfile.gettempdir(), "prl_franka_mujoco")

    use_rviz_bool = context.perform_substitution(use_rviz).lower() == "true"

    on_aux_computer_bool = (
        context.perform_substitution(on_aux_computer).lower() == "true"
    )
    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )
    external_controllers_names_list = ast.literal_eval(
        context.perform_substitution(external_controllers_names)
    )

    if robot_ip_empty and not use_sim_time:
        raise RuntimeError(
            "Incorrect launch configuration! Set `robot_ip` to configure hardware or "
            "`use_gazebo:=true` / `use_mujoco:=true` to configure simulation."
        )

    if not robot_ip_empty and use_sim_time:  # If simulation is used, ignore robot_ip
        robot_ip = ""

    if robot_ip_empty and disable_collision_safety_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`disable_collision_safety:=true` and empty `robot_ip`. "
            "Disabling collision safety is only supported on the real robot."
        )

    if robot_ip_empty and not aux_computer_ip_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "non empty `aux_computer_ip` and empty `robot_ip`."
        )

    if not aux_computer_ip_empty and aux_computer_user_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "non empty `aux_computer_ip` and empty `aux_computer_user`."
        )

    if robot_ip_empty and on_aux_computer_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and non-empty `robot_ip`."
        )

    if not aux_computer_ip_empty and on_aux_computer_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and non-empty `aux_computer_ip_empty`."
        )

    if on_aux_computer_bool and use_sim_time:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_gazebo:=true`."
        )

    if on_aux_computer_bool and use_rviz_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_rviz:=true`."
        )

    franka_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_franka_run"),
                        "launch",
                        "real.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "ee_id": ee_id,
            "franka_controllers_params": franka_controllers_params,
            "franka_controllers_setup": franka_controllers_setup,
            "ft_sensor": use_ft_sensor,
            "tare_ft_sensor": tare_ft_sensor,
            "activate_cameras": activate_cameras,
        }.items(),
        condition=UnlessCondition(
            PythonExpression([str(use_sim_time), " or '", aux_computer_ip, "' != ''"])
        ),
    )

    franka_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("prl_franka_run"),
                        "launch",
                        "sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_verbose": gz_verbose,
            "gz_headless": gz_headless,
            "gz_world_path": gz_world_path,
            "use_ft_sensor": use_ft_sensor,
            "ee_id": ee_id,
            "load_end_effector": load_end_effector,
            "franka_controllers_setup": franka_controllers_setup,
            "activate_cameras": activate_cameras,
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_moveit"),
                    "launch",
                    "start_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
        condition=IfCondition(use_moveit),
    )

    def _to_subst_value(val):
        if isinstance(val, (list, tuple)):
            return " ".join(map(str, val))
        return val

    xacro_args = {
        "ros2_control": "true",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "gazebo": use_gazebo,
        "gazebo_effort": use_gazebo,
        "with_sc": "false",
        "initial_joint_position": initial_joint_position,
        "use_ft_sensor": use_ft_sensor,
        "robot_config_file": config_file,
        "activate_cameras": activate_cameras,
        "mujoco": str(use_mujoco_bool).lower(),
        "mujoco_model": os.path.join(mujoco_dir, "scene.xml"),
        "mujoco_headless": LaunchConfiguration("mujoco_headless"),
    }

    robot_description_file_substitution = PathJoinSubstitution(
        [
            FindPackageShare("prl_franka_description"),
            "urdf",
            "prl_franka.urdf.xacro",
        ]
    )
    robot_description_command = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_file_substitution,
            # Convert dict to list of parameters
            *[arg for key, val in xacro_args.items() for arg in (f" {key}:=", val)],
        ]
    )
    if use_gazebo_bool:
        # Gazebo only: Franka's 0.2 Nm joint Coulomb friction causes stick-slip vibrations of the wrist
        # with bullet-featherstone (PD effort control, no integral). Real robot never reads this URDF's physics.
        urdf = context.perform_substitution(robot_description_command)
        robot_description = ParameterValue(
            re.sub(r'friction="[^"]*"', 'friction="0.0"', urdf), value_type=str
        )
    elif use_mujoco_bool:
        # sim_mujoco.launch.py builds the MJCF scene from this exact URDF
        urdf = context.perform_substitution(robot_description_command)
        os.makedirs(mujoco_dir, exist_ok=True)
        with open(os.path.join(mujoco_dir, "robot_description.urdf"), "w") as f:
            f.write(urdf)
        robot_description = ParameterValue(urdf, value_type=str)
    else:
        robot_description = ParameterValue(robot_description_command, value_type=str)

    mujoco_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_mujoco"),
                    "launch",
                    "sim_mujoco.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_description_file": os.path.join(
                mujoco_dir, "robot_description.urdf"
            ),
            "mujoco_scene": os.path.join(mujoco_dir, "scene.xml"),
            "franka_controllers_setup": franka_controllers_setup,
            "ee_id": ee_id,
            "load_end_effector": load_end_effector,
        }.items(),
        condition=IfCondition(use_mujoco),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--display-config", rviz_config_path],
        # MoveIt opens its own RViz
        condition=IfCondition(
            PythonExpression(
                ["'", use_rviz, "' == 'true' and '", use_moveit, "' != 'true'"]
            )
        ),
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_plotjuggler")),
        arguments=["--layout", plotjuggler_config_file],
    )

    return [
        franka_hardware_launch,
        franka_simulation_launch,
        mujoco_simulation_launch,
        robot_state_publisher_node,
        rviz_node,
        plotjuggler_node,
        moveit_launch,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_configuration"),
                    "config",
                    "robot_config.yaml",
                ]
            ),
            description="Path to the robot configuration file.",
        ),
        DeclareLaunchArgument(
            "external_controllers_params",
            default_value="",
            description="Path to the yaml file use to define external controllers parameters.",
        ),
        DeclareLaunchArgument(
            "external_controllers_names",
            default_value="['']",
            description="List of names of the external controllers to spawn.",
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_control"),
                    "config/arm",
                    "arm_controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "franka_controllers_setup",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_configuration"),
                    "config",
                    "controller_setup.yaml",
                ]
            ),
            description="Path to the yaml file used to define which controllers to load and activate.",
        ),
        DeclareLaunchArgument(
            "initial_joint_position",
            default_value="'0.0 -0.78 0.0 -2.35 0.0 1.57 0.78 0.0'",
            description="Initial configuration of the robot used in Gazebo",
        ),
        DeclareLaunchArgument(
            "gz_world_path",
            default_value="",
            description="Gazebo world SDF. Empty: prl_franka_description/worlds/prl_franka.sdf "
            + "(Franka's sensors world with bullet-featherstone physics).",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_description"),
                    "rviz",
                    "config.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
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
            "aux_computer_ip",
            default_value="",
            description="Hostname or IP address of the auxiliary computer "
            + "with real-time kernel. If not empty launch file is configured "
            + "to spawn docker container on that machine. If empty, controllers "
            + "are spawned locally on the computer executing launch file.",
        ),
        DeclareLaunchArgument(
            "aux_computer_user",
            default_value="",
            description="Username used to execute commands on auxiliary computer over ssh. "
            + "Required if `aux_computer_ip` is not empty.",
        ),
        DeclareLaunchArgument(
            "on_aux_computer",
            default_value="false",
            description="Whether launch file is executed on auxiliary computer. "
            + "If set to `true`, `robot_ip` can not be empty and only minimal "
            + "set of nodes to control the robot is launched on this machine.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_mujoco",
            default_value="false",
            description="Simulate the robot in MuJoCo (mujoco_ros2_control). The MJCF scene is "
            + "generated from the URDF (needs the prl_franka_mujoco package).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "mujoco_headless",
            default_value="false",
            description="Run MuJoCo without its viewer window.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Configures launch file for Gazebo simulation. "
            + "If set to `true` launch file is configured for simulated robot. "
            + "If set to `false` argument `robot_ip` is expected not to be empty.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_ft_sensor",
            default_value="false",
            description="Enable or disable use of force-torque sensor",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "tare_ft_sensor",
            default_value="false",
            description="Whether to tare the force torque sensor on startup.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Whether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Whether to launch Gazebo in headless mode "
            + "(no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "activate_cameras",
            default_value="false",
            description="Turn on the fixed cameras: Gazebo streams in simulation, "
            + "drivers (prl_franka_control/sensors.launch.py) on the real robot.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_moveit",
            default_value="false",
            description="Launch MoveIt (move_group + its RViz). Needs fr3_arm_controller active "
            + "(prl_franka_configuration/config/controller_setup.yaml).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_plotjuggler",
            default_value="false",
            description="Whether to launch PlotJuggler for real-time plotting of robot data.",
            choices=["true", "false"],
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
