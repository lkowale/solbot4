#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
