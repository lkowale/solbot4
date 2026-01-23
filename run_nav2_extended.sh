#!/bin/bash
# Run Nav2 simulation with extended config (includes move_sequence behavior)

set -e

# Kill any existing ROS2 and Gazebo processes
echo "Cleaning up existing processes..."
pkill -f "ros2" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
sleep 2

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws4/install/setup.bash

# Default settings
HEADLESS="${HEADLESS:-True}"
USE_RVIZ="${USE_RVIZ:-False}"

# Get the params file path
PARAMS_FILE=$(ros2 pkg prefix solbot4_nav2_bringup)/share/solbot4_nav2_bringup/params/nav2_params_extended.yaml

echo "Starting Nav2 simulation with extended config..."
echo "  Params file: $PARAMS_FILE"
echo "  Headless: $HEADLESS"
echo "  Use RViz: $USE_RVIZ"
echo ""
echo "Available actions after launch:"
echo "  - /move (opennav_coverage_msgs/action/Move)"
echo "  - /move_sequence (opennav_coverage_msgs/action/MoveSequence)"
echo ""

ros2 launch solbot4_nav2_sim nav2_simulation.launch.py \
    headless:=$HEADLESS \
    use_rviz:=$USE_RVIZ \
    params_file:=$PARAMS_FILE \
    "$@"
