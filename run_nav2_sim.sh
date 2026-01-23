#!/bin/bash
# Run solbot4 Nav2 simulation

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws4/install/setup.bash

# Default arguments
HEADLESS="${HEADLESS:-True}"
USE_RVIZ="${USE_RVIZ:-False}"

# Launch Nav2 simulation
ros2 launch solbot4_nav2_sim nav2_simulation.launch.py \
    headless:=$HEADLESS \
    use_rviz:=$USE_RVIZ \
    "$@"
