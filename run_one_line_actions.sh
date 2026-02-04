#!/bin/bash
# Run Nav2 simulation with OneLine Navigator using Standalone Action Servers
#
# Uses standalone action servers instead of Nav2 navigator plugins:
#   - /approach_swath (approach_swath_action)
#   - /run_swath (run_swath_action)
#   - /move_sequence (move_sequence_behavior - Nav2 behavior)
#
# Supports pause/resume via /pause topic
#
# After launch, execute with coordinates:
#   ros2 action send_goal /run_one_line solbot4_msgs/action/RunOneLine "{start_point: {latitude: 53.5204497, longitude: 17.8259001}, end_point: {latitude: 53.5203518, longitude: 17.8258440}}" --feedback
#
# Or from file:
#   ros2 action send_goal /run_one_line solbot4_msgs/action/RunOneLine "{field_name: 'yard_one_line'}" --feedback
#
# To pause:
#   ros2 topic pub --once /pause solbot4_msgs/msg/Pause "{paused: true, source: 'operator', reason: 'Manual pause'}"
#
# To resume:
#   ros2 topic pub --once /pause solbot4_msgs/msg/Pause "{paused: false, source: 'operator', reason: 'Resuming'}"

set -e

# Use CycloneDDS for more reliable action server communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Kill any existing ROS2 and Gazebo processes
echo "Cleaning up existing processes..."
pkill -f "ros2" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "approach_swath_action" 2>/dev/null || true
pkill -f "run_swath_action" 2>/dev/null || true
sleep 2

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws4/install/setup.bash

# Default settings
HEADLESS="${HEADLESS:-True}"
USE_RVIZ="${USE_RVIZ:-False}"
USE_MAPVIZ="${USE_MAPVIZ:-False}"

# Get the params file path
PARAMS_FILE=$(ros2 pkg prefix solbot4_nav2_bringup)/share/solbot4_nav2_bringup/params/nav2_params_one_line_actions.yaml

echo "=========================================="
echo "  OneLine Navigator (Standalone Actions)"
echo "=========================================="
echo ""
echo "Params file: $PARAMS_FILE"
echo "Headless: $HEADLESS"
echo "Use RViz: $USE_RVIZ"
echo "Use Mapviz: $USE_MAPVIZ"
echo ""
echo "Starting standalone action servers..."
echo "=========================================="

# Start standalone action servers in background
ros2 run approach_swath_action approach_swath_action_node &
APPROACH_PID=$!
echo "Started approach_swath_action_node (PID: $APPROACH_PID)"

ros2 run run_swath_action run_swath_action_node &
RUN_SWATH_PID=$!
echo "Started run_swath_action_node (PID: $RUN_SWATH_PID)"

# Give action servers time to initialize
sleep 2

echo ""
echo "Starting Nav2 simulation..."
echo "=========================================="

# Cleanup function to kill background processes on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $APPROACH_PID 2>/dev/null || true
    kill $RUN_SWATH_PID 2>/dev/null || true
    pkill -f "approach_swath_action" 2>/dev/null || true
    pkill -f "run_swath_action" 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch solbot4_nav2_sim nav2_simulation.launch.py \
    headless:=$HEADLESS \
    use_rviz:=$USE_RVIZ \
    use_mapviz:=$USE_MAPVIZ \
    params_file:=$PARAMS_FILE \
    "$@"
