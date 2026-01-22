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
    config_file = LaunchConfiguration("robot_config_file").perform(context)
    config_path = Path(config_file)
    with config_path.open("r") as setup_file:
        robot = yaml.safe_load(setup_file)
        config = robot.get("ROBOT", {})
    arm_id = config.get("arm_id", "fr3")
    if arm_id not in ["fer", "fr3", "fp3"]:
        raise RuntimeError(
            f"Unsupported arm_id '{arm_id}' in robot configuration file! "
            "Supported values are: 'fer', 'fr3', 'fp3'."
        )
    arm_prefix = config.get("arm_prefix", "")
    robot_ip = config.get("robot_ip", "")
    namespaces = config.get("namespaces", "")
    ee_id = config.get("ee_id", "franka_hand")
    load_gripper = config.get("load_gripper", "true")
    gripper_type = config.get("gripper_type", "franka_gripper")
    with_sc = config.get("self_collision_safety", "true")
    ee_pose = config.get("end_effector_pose", {})

    aux_computer_ip = LaunchConfiguration("aux_computer_ip")
    aux_computer_user = LaunchConfiguration("aux_computer_user")
    on_aux_computer = LaunchConfiguration("on_aux_computer")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_sim_time = LaunchConfiguration("use_gazebo")
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")
    gz_world_path = LaunchConfiguration("gz_world_path")

    external_controllers_params = LaunchConfiguration("external_controllers_params")
    external_controllers_names = LaunchConfiguration("external_controllers_names")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    franka_controllers_setup = LaunchConfiguration("franka_controllers_setup")
    initial_joint_position = LaunchConfiguration("initial_joint_position")

    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    ft_sensor_ip = LaunchConfiguration("ft_sensor_ip")

    robot_ip_empty = robot_ip == ""
    aux_computer_ip_empty = context.perform_substitution(aux_computer_ip) == ""
    aux_computer_user_empty = context.perform_substitution(aux_computer_user) == ""
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"

    use_rviz_bool = context.perform_substitution(use_rviz).lower() == "true"

    use_ft_sensor_bool = context.perform_substitution(use_ft_sensor).lower() == "true"
    ft_sensor_ip_empty = context.perform_substitution(ft_sensor_ip) == ""
    on_aux_computer_bool = (
        context.perform_substitution(on_aux_computer).lower() == "true"
    )
    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )
    external_controllers_names_list = ast.literal_eval(
        context.perform_substitution(external_controllers_names)
    )

    if robot_ip_empty and not use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Set `robot_ip` to configure hardware or "
            "`use_gazebo:=true` to configure simulation."
        )

    if not robot_ip_empty and use_gazebo_bool:  # If simulation is used, ignore robot_ip
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

    if on_aux_computer_bool and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_gazebo:=true`."
        )

    if on_aux_computer_bool and use_rviz_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_rviz:=true`."
        )

    if use_ft_sensor_bool and ft_sensor_ip_empty and not robot_ip_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "`use_ft_sensor:=true` empty `ft_sensor_ip` and non-empty `robot_ip`."
        )

    if not ft_sensor_ip_empty and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "non empty `ft_sensor_ip` and `use_gazebo:=true`."
        )

    if not use_ft_sensor_bool and ee_id == "ati_mini45_with_camera":
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "`use_ft_sensor:=false` and `ee_id:=ati_mini45_with_camera`."
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
        }.items(),
        condition=UnlessCondition(
            OrSubstitution(
                use_gazebo, PythonExpression(["'", aux_computer_ip, "' != ''"])
            )
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
        }.items(),
        condition=IfCondition(use_gazebo),
    )
    print(f"load gripper: {load_gripper}")

    def _to_subst_value(val):
        if isinstance(val, (list, tuple)):
            return " ".join(map(str, val))
        return val

    xacro_args = {
        "robot_ip": robot_ip,
        "arm_id": arm_id,
        "arm_prefix": arm_prefix,
        "ros2_control": "true",
        "hand": load_gripper,
        "gripper_type": gripper_type,
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "gazebo": use_gazebo,
        "ee_id": ee_id,
        "gazebo_effort": use_gazebo,
        "with_sc": "false",
        "initial_joint_position": initial_joint_position,
        "with_sc": with_sc,
        "xyz_ee": f"'{_to_subst_value(ee_pose.get('xyz', '0 0 0'))}'",
        "rpy_ee": f"'{_to_subst_value(ee_pose.get('rpy', '0 0 0'))}'",
    }

    robot_description_file_substitution = PathJoinSubstitution(
        [
            FindPackageShare("prl_franka_description"),
            "urdf",
            "prl_franka.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                robot_description_file_substitution,
                # Convert dict to list of parameters
                *[arg for key, val in xacro_args.items() for arg in (f" {key}:=", val)],
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "source_list": [
                    "franka/joint_states",
                    f"{arm_id}_gripper/joint_states",
                ],
                "rate": 30,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return [
        franka_hardware_launch,
        franka_simulation_launch,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("prl_franka_run"),
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
            "initial_joint_position",
            default_value="'0.0 -0.78 0.0 -2.35 0.0 1.57 0.78 0.0'",
            description="Initial configuration of the robot used in Gazebo",
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
            "ft_sensor_ip",
            default_value="",
            description="Hostname or IP address of the force-torque sensor.",
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
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
