#!/bin/bash

colcon build --symlink-install --cmake-args -DPython_EXECUTABLE=$(which python)
source install/setup.bash
