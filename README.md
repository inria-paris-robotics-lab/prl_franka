# PRL Franka Research 3 ROS 2 Packages

This project integrates a ROS 2 development environment with Pixi and provides packages for the description, simulation, and control of the **Franka FR3** robot, developed by the **Paris Robotics Lab**.

## Included Packages

| Package | Description |
|---|---|
| prl_fr3_description | Provides the robot description, including 3D assets (meshes, URDF/Xacro) required to visualize and simulate the FR3 in a ROS 2 environment. |
| prl_fr3_gazebo | Provides launch files and configurations needed to simulate the FR3 in Gazebo. |
| prl_fr3_control | Provides configuration files for ROS 2 controllers (ros2_control) and launch files to deploy the FR3 controllers. |
| prl_fr3_moveit | Provides configuration and launch files to control the FR3 using various motion-planning solvers with MoveIt 2. |
| prl_fr3_run | Provides high-level launch files for accessing the real robot (starting the franka_ros2 driver and enabling control) or launching full simulations. |

## Prerequisites

*   **Pixi** must be installed on your machine.
*   **Pre-commit** installed for code quality checks.

---

## Installation

Clone the repository and set up the workspace by following the steps below.

```bash
# Clone the repository
git clone
cd prl_fr3
```

### 1. Environment Setup

Build your Pixi environment with the required dependencies:

```bash
# Enable insecure post-link scripts to allow ros autocompletion
pixi config set --local run-post-link-scripts insecure
# Install dependencies and build the environment
pixi install
```
Launch the Pixi environment:

```bash
pixi shell
```
### 2. Build the ros2 workspace

Follow the steps below to set up and build the ROS 2 workspace.

```bash
cd ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



## Configurate your setup with the real robot

Before using the FR3, you need to configure your hardware setup.

### prl_fr3_robot_configuration
Edit the `prl_fr3_robot_configuration/config/standard_setup.yaml` file (or equivalent). Update the following parameters:

*   **Robot IP:** Specify the IP address of the FR3 Control station (usually `172.16.0.2` or `192.168.1.1`).
*   **Cameras:** Configure hand-eye or fixed cameras (Realsense, Orbbec).
*   **Gripper:** Define if the standard Franka Hand or a custom gripper is used.

---

## Usage Tips

> **Note:** The following instructions are examples. Use `--show-args` on launch files to see all options.

### Source the workspace
Before running any launch files, source the workspace:

```bash
source install/setup.bash
```

### 1. Visualize in RViz
Only visualize the robot model stabdalone:

```bash
ros2 launch prl_fr3_description view_robot.launch.py
```

### 2. Simulation (Gazebo + MoveIt)
To simulate the FR3 in Gazebo and control it via MoveIt:

```bash
 ros2 launch prl_franka_run franka.launch.py use_gazebo:=true use_rviz:=true
```

### 3. Real Robot
To control the real FR3 hardware:

1.  Ensure the robot brakes are unlocked and FCI is activated (blue light).
2.  Launch the driver and controllers:

```bash
ros2 launch prl_fr3_run franka.launch.py robot_ip:=<ROBOT_IP_ADDRESS> use_rviz:=true
```

---

## Important Notes

*   **Real-Time Kernel:** Controlling the physical FR3 often requires a Linux kernel with PREEMPT_RT patches for stable communication (`libfranka`). Ensure your host machine (or Docker setup) supports this if you experience jerky motion or communication errors.
*   **Network:** Ensure your machine is on the same subnet as the robot.
