if command -v register-python-argcomplete >/dev/null 2>&1; then
    eval "$(register-python-argcomplete ros2)"
    eval "$(register-python-argcomplete colcon)"
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix prl_franka_description)/share:$(ros2 pkg prefix franka_description)/share
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(ros2 pkg prefix franka_ign_ros2_control)/lib

    echo "[DEBUG] ROS 2 completion enabled."
else
    echo "[DEBUG] argcomplete not found!"
fi