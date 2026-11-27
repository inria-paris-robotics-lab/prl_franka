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
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetParameter
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def camera_launches(cameras_config):
    """Drivers for the cameras with `activate: true` in the cameras config file."""
    # cameras_config: file name in prl_franka_configuration/config/fixed_cameras, or an absolute path
    config_file = (
        Path(get_package_share_directory("prl_franka_configuration"))
        / "config/fixed_cameras"
        / cameras_config
    )
    with config_file.open("r") as setup_file:
        cameras = (yaml.safe_load(setup_file) or {}).get("cameras", {})

    realsense_cameras, primary_cameras, secondary_cameras = [], [], []
    for camera_name, camera_info in cameras.items():
        if not camera_info.get("activate", False):
            continue
        if camera_info.get("type") == "realsense":
            realsense_cameras.append((camera_name, camera_info))
        elif camera_info.get("type") == "femto-mega":
            if camera_info.get("sync_mode") == "primary":
                primary_cameras.append((camera_name, camera_info))
            else:
                secondary_cameras.append((camera_name, camera_info))

    def b(info, key, default):
        return str(info.get(key, default)).lower()

    def orbbec_node(camera_name, camera_info):
        sync_mode = camera_info.get("sync_mode", "standalone")
        camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("orbbec_camera"), "/launch", "/femto_mega.launch.py"]
            ),
            launch_arguments={
                # --- CAMERA IDENTIFICATION ---
                "camera_name": camera_name,
                "net_device_ip": str(camera_info.get("ip")),
                "net_device_port": str(camera_info.get("port")),
                "enumerate_net_device": "false",
                # --- HARDWARE SYNC ---
                "sync_mode": sync_mode,
                "enable_frame_sync": b(camera_info, "enable_frame_sync", True),
                "depth_delay_us": str(camera_info.get("depth_delay_us", 0)),
                "color_delay_us": str(camera_info.get("color_delay_us", 0)),
                "trigger2image_delay_us": str(
                    camera_info.get("trigger2image_delay_us", 0)
                ),
                "trigger_out_delay_us": str(camera_info.get("trigger_out_delay_us", 0)),
                "trigger_out_enabled": "true" if sync_mode == "primary" else "false",
                # --- SOFTWARE SYNC ---
                "time_domain": str(camera_info.get("time_domain", "system")),
                # --- FLUX AND RESOLUTION ---
                "enable_color": b(camera_info, "enable_color", True),
                "enable_depth": b(camera_info, "enable_depth", True),
                "enable_ir": b(camera_info, "enable_ir", False),
                "color_width": str(camera_info.get("color_width", 1280)),
                "color_height": str(camera_info.get("color_height", 720)),
                "depth_width": str(camera_info.get("depth_width", 640)),
                "depth_height": str(camera_info.get("depth_height", 576)),
                "color_fps": str(camera_info.get("color_fps", 15)),
                "depth_fps": str(camera_info.get("depth_fps", 15)),
                "ir_fps": str(camera_info.get("ir_fps", 15)),
                "enable_point_cloud": b(camera_info, "enable_point_cloud", True),
                "enable_colored_point_cloud": b(
                    camera_info, "enable_colored_point_cloud", False
                ),
                "depth_registration": b(camera_info, "depth_registration", False),
                "publish_tf": "false",
                # --- FILTERING ---
                "enable_decimation_filter": b(
                    camera_info, "enable_decimation_filter", False
                ),
                "enable_spatial_filter": b(camera_info, "enable_spatial_filter", False),
                "spatial_filter_alpha": str(
                    camera_info.get("spatial_filter_alpha", -1.0)
                ),
                "spatial_filter_magnitude": str(
                    camera_info.get("spatial_filter_magnitude", -1)
                ),
                "spatial_filter_radius": str(
                    camera_info.get("spatial_filter_radius", -1)
                ),
                "enable_temporal_filter": b(
                    camera_info, "enable_temporal_filter", False
                ),
                "temporal_filter_weight": str(
                    camera_info.get("temporal_filter_weight", -1.0)
                ),
                "temporal_filter_diff_threshold": str(
                    camera_info.get("temporal_filter_diff_threshold", -1.0)
                ),
                "enable_hole_filling_filter": b(
                    camera_info, "enable_hole_filling_filter", False
                ),
                "hole_filling_filter_mode": str(
                    camera_info.get("hole_filling_filter_mode", "")
                ),
                "enable_threshold_filter": b(
                    camera_info, "enable_threshold_filter", False
                ),
                "threshold_filter_max": str(
                    camera_info.get("threshold_filter_max", -1)
                ),
                "threshold_filter_min": str(
                    camera_info.get("threshold_filter_min", -1)
                ),
                "enable_noise_removal_filter": b(
                    camera_info, "enable_noise_removal_filter", False
                ),
                "noise_removal_filter_min_diff": str(
                    camera_info.get("noise_removal_filter_min_diff", 10)
                ),
                "noise_removal_filter_max_size": str(
                    camera_info.get("noise_removal_filter_max_size", 50)
                ),
            }.items(),
        )
        return GroupAction(
            actions=[
                PushRosNamespace("camera"),
                SetParameter(name="enable_sync_host_time", value=False),
                camera_node,
            ]
        )

    launches = []
    for camera_name, camera_info in realsense_cameras:
        if not camera_info.get("serial_no"):
            continue
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("realsense2_camera"), "/launch", "/rs_launch.py"]
                ),
                launch_arguments={
                    "camera_name": camera_name,
                    "serial_no": camera_info["serial_no"],
                    "enable_color": b(camera_info, "enable_color", False),
                    "enable_depth": b(camera_info, "enable_depth", False),
                    "pointcloud.enable": b(camera_info, "pointcloud", False),
                    "enable_infra": b(camera_info, "enable_infra", False),
                    "enable_infra1": b(camera_info, "enable_infra1", False),
                    "enable_infra2": b(camera_info, "enable_infra2", False),
                    "enable_gyro": b(camera_info, "enable_gyro", False),
                    "enable_accel": b(camera_info, "enable_accel", False),
                    "enable_sync": b(camera_info, "enable_sync", False),
                    "align_depth.enable": b(camera_info, "align_depth", False),
                    "enable_rgbd": b(camera_info, "enable_rgbd", False),
                }.items(),
            )
        )

    # Orbbec cameras must start 2 s apart, the primary (sync master) last
    delay = 0.0
    for camera_name, camera_info in secondary_cameras + primary_cameras:
        launches.append(
            TimerAction(period=delay, actions=[orbbec_node(camera_name, camera_info)])
        )
        delay += 2.0
    return launches


def launch_setup(context):
    # Initialize Arguments
    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    tare_ft_sensor = LaunchConfiguration("tare_ft_sensor")
    activate_cameras = (
        LaunchConfiguration("activate_cameras").perform(context) == "true"
    )
    cameras_config = LaunchConfiguration("cameras_config").perform(context)
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
    return (camera_launches(cameras_config) if activate_cameras else []) + [
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_cameras",
            description="Launch the drivers of the cameras activated in cameras_config.",
            default_value="false",
            choices=["true", "false"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cameras_config",
            description="File in prl_franka_configuration/config/fixed_cameras, or an absolute path.",
            default_value="cameras_config.yaml",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
