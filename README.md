# PRL Franka — ROS 2 Setup

This project provides a complete **ROS 2 development environment** based on **Pixi**, including packages for **description, simulation, control, and motion planning** of the **Franka Robot** robot, developed by the **Paris Robotics Lab (PRL)**.

---

## Included Packages

| Package               | Description                                                                                                    |
| --------------------- | -------------------------------------------------------------------------------------------------------------- |
| `prl_fr3_description` | Robot description package containing URDF/Xacro files and 3D assets (meshes) for visualization and simulation. |
| `prl_fr3_control`     | ROS 2 control (`ros2_control`) configuration files and controller launch utilities.                            |
| `prl_fr3_moveit`      | MoveIt 2 configuration and launch files for motion planning with the FR3.                                      |
| `prl_fr3_run`         | High-level launch files for running the FR3 on real hardware or in full simulation.                            |

---

## Prerequisites

* **Pixi** installed on your system
* **Pre-commit** installed for code quality checks

---

## Installation

Clone the repository and set up the workspace:

```bash
git clone https://github.com/inria-paris-robotics-lab/prl_franka.git
cd prl_franka
```

---

## 1. Environment Setup (Pixi)

Build the Pixi environment and install dependencies:

> [!NOTE]
> The `run-post-link-scripts insecure` setting is required to allow ROS setup scripts to run.*

```bash
# Allow post-link scripts (needed for ROS autocompletion)
pixi config set --local run-post-link-scripts insecure

# Install dependencies
pixi install
```

Activate the Pixi environment:
> [!NOTE]
> For the first launch, this may take a while as Pixi builds the ros2 workspace from source.
```bash
pixi shell
```

---

## 2. Build the ROS 2 Workspace

Normally, Pixi should automatically build the ROS 2 workspace on the first launch. If you need to rebuild manually:

```bash
cd ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Configure the Real Robot Setup

Before using the FRANKA hardware, configure your setup.


Edit the file: `prl_franka_run/config/robot_config.yaml`

## Robot Configuration File (`robot_config.yaml`)

| Field                   | Type / Example     | Purpose                                                                 |
| ----------------------- | ------------------ | ----------------------------------------------------------------------- |
| `ROBOT`                 | section            | Root section describing one robot instance.                             |
| `arm_id`                | `"fr3"`            | Robot model / arm identifier (reserved for future multi-robot support). |
| `arm_prefix`            | `""`               | Optional unique robot prefix (reserved for future use).                 |
| `namespace`             | `""`               | ROS namespace. Should be unique to avoid topic conflicts.               |
| `robot_ip`              | `"172.17.1.5"`     | IP or hostname of the real robot (ignored in simulation).               |
| `load_end_effector`          | `"true"`           | Whether to load the end effector model.                                      |
| `end_effector_pose`     | section            | Pose offset applied to the end-effector frame.                          |
| `end_effector_pose.xyz` | `[0.0, 0.0, 0.01]` | Position offset `[x, y, z]` in meters.                                  |
| `end_effector_pose.rpy` | `[0.0, 0.0, 0.0]`  | Orientation offset `[roll, pitch, yaw]` in radians.                     |
| `ee_id`                 | `"ball"`           | End-effector type (`franka_hand`, `ball`, etc.).                        |
| `self_collision_safety` | `"false"`          | Enable or disable self-collision safety checks.                         |
| `use_ft_sensor`      | `"true"`                    | Enable force-torque sensor (`"true"` or `"false"`).                                         |


---

## Usage

> [!NOTE]
> These are example commands. Use `--show-args` on launch files to see all available options.

### Source the Workspace

```bash
source install/setup.bash
```

---

### 1. Visualize the Robot in RViz

Visualize the robot model only (no simulation, no controllers):

```bash
ros2 launch prl_franka_description view_robot.launch.py
```

---

### 2. Simulation (Gazebo + RViz)

Simulate the robot in Gazebo with visualization:

```bash
ros2 launch prl_franka_run franka.launch.py use_gazebo:=true use_rviz:=true
```

---

### 3. Real Robot Execution

To control the real robot:

1. Ensure the robot brakes are released and the FCI is active (blue light).
2. Launch the driver and controllers:

```bash
ros2 launch prl_franka_run franka.launch.py use_gazebo:=false use_rviz:=true
```

---

## Launch Arguments

| Argument                      | Default value                                     | Purpose                                                                         |
| ----------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------- |
| `robot_config_file`           | `prl_franka_run/config/robot_config.yaml`         | Main robot configuration file (arm ID, end-effector, IPs, gripper, namespaces). |
| `external_controllers_params` | `""`                                              | YAML file defining external/custom controllers parameters.                      |
| `external_controllers_names`  | `['']`                                            | List of external controller names to spawn.                                     |
| `franka_controllers_params`   | `prl_franka_control/config/controllers.yaml`      | Parameters for Franka arm controllers (gains, joints, interfaces).              |
| `franka_controllers_setup`    | `prl_franka_control/config/controller_setup.yaml` | Defines which controllers are loaded and activated.                             |
| `initial_joint_position`      | `'0.0 -0.78 0.0 -2.35 0.0 1.57 0.78 0.0'`         | Initial joint configuration used in Gazebo simulation.                          |
| `gz_world_path`               | `prl_franka_description/worlds/empty.sdf`         | Gazebo world (SDF) to load.                                                     |
| `rviz_config_path`            | `prl_franka_description/rviz/config.rviz`         | RViz configuration file.                                                        |
| `use_gazebo`                  | `false`                                           | Enable Gazebo simulation mode.                                                  |
| `use_rviz`                    | `false`                                           | Launch RViz for visualization.                                                  |
| `gz_verbose`                  | `false`                                           | Enable verbose Gazebo logging.                                                  |
| `gz_headless`                 | `false`                                           | Run Gazebo without GUI (physics server only).                                   |
| `use_ft_sensor`               | `false`                                           | *Not supported yet.*                                                            |
| `ft_sensor_ip`                | `""`                                              | *Not supported yet.*                                                            |


---
> [!IMPORTANT]
> **Real-Time Kernel**
> Running the real robot reliably often requires a Linux kernel with **PREEMPT_RT** patches for stable `libfranka` communication.
>
> **Network Configuration**
> Ensure your machine is on the same subnet as the robot to avoid connectivity issues.

---
