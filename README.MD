FRANKA PRL 
===

## Summary
Generate and execute quadruped motion on a real Go2 robot mainly using Aligator (for Trajectory Optimization) and Simple-MPC (for Model Predictive Control).
This project also relies on our go2_odometry package and ROS robot_localization package for state estimation

## Installation
The following procedure allow for installing all the dependencies for Aligator, Simple-MPC, ... all together.

1. Clone this repo
```bash
git clone 
```

2. Create conda environment.
(It is recommended to use `mamba/micro` instead of `conda` for faster/better dependencies solving)
```bash
mamba env create -f prl_franka/environment.yaml
mamba activate franka-prl
```

4. Clone some dependencies
(Some dependencies are not available on conda, or not with adequate versions)
(vcs allow for cloning and managing multiple repo at once)
```bash
vcs import --recursive < go2_mpc_experiments/git-deps.yaml
```

5. Build all CMake packages
* A. Move to workspace root directory
```bash
cd ..
```

* B. Build those two packages alone first, and source them (required by unitree install procedure)

```bash
colcon build --event-handlers console_direct+ --packages-select cyclonedds GeographicLib
source install/setup.bash
export CYCLONEDDS_HOME=$(pwd)"/install/cyclonedds"
```

* C. Build all remaining packages

```bash
export MAKEFLAGS="-j4" # It is recommended to reduce the number of jobs as you ram might get full easily with the default number.
colcon build --event-handlers console_direct+ --packages-ignore unitree_sdk2_python --cmake-args \
-DCMAKE_BUILD_TYPE=Release             \
-DCMAKE_PREFIX_PATH=$CONDA_PREFIX      \
-DPYTHON_EXECUTABLE=$(which python)    \
-DCMAKE_CXX_COMPILER_LAUNCHER='ccache' \
-DBUILD_TESTING=OFF                    \
-DBUILD_DOCUMENTATION=OFF              \
-DBUILD_EXAMPLES=OFF                   \
-DBUILD_BENCHMARK=OFF                  \
-DBUILD_BENCHMARKS=OFF                 \
-DBUILD_WITH_COLLISION_SUPPORT=ON      \
-DBUILD_WITH_PROXQP=ON                 \
-DGENERATE_PYTHON_STUBS=OFF            \
-DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON
```

6. Source the environment to run on local
```bash
mamba activate go2-mpc # If not already done
source install/setup.bash
source <(ros2 run go2_control_interface autoset_environment_dds.py SIMULATION)
```
Source the environment to run on the robot
```bash
mamba activate go2-mpc # If not already done
source install/setup.bash
source <(ros2 run go2_control_interface autoset_environment_dds.py REAL)
```

7. Build unitree_sdk
```bash
cd src/unitree_sdk2_python
pip install -e .
```

## Launch file
#### Connect to the robot:
For real robot:
```bash
ros2 launch go2_mpc_experiments real.launch.py
```
For simulation:
```bash
ros2 launch go2_mpc_experiments sim.launch.py
```

#### Start the mpc
```bash
ros2 launch go2_mpc_experiments mpc.launch.py
```
(launch a joystick node aswell)

### Details
The above launchfiles will generate the following ros graph
![nodes_flowchart](ressources/flowchart.svg)