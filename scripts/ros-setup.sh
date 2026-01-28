#!/bin/bash

# Build workspace if not built yet
if [ ! -f "ws/install/setup.bash" ]; then
    echo "[INFO] ROS 2 workspace not built yet. Building..."
    cd ws
    colcon build --symlink-install
    cd ..
fi

# Source setup ROS 2
source "ws/install/setup.bash"

export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(ros2 pkg prefix franka_ign_ros2_control)/lib
PRL_FRANKA_PATH="$(ros2 pkg prefix prl_franka_description)/share"
FRANKA_PATH="$(ros2 pkg prefix franka_description)/share"
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PRL_FRANKA_PATH:$FRANKA_PATH"
echo "[DEBUG] GZ_SIM_RESOURCE_PATH: $GZ_SIM_RESOURCE_PATH"

# Autocompletion ROS 2
if command -v register-python-argcomplete >/dev/null 2>&1; then
    eval "$(register-python-argcomplete ros2)"
    eval "$(register-python-argcomplete colcon)"
    echo "[DEBUG] ROS 2 completion enabled."
else
    echo "[DEBUG] argcomplete not found!"
fi

# Place the user in the workspace directory
cd ws || exit 1
