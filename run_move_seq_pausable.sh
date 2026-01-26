#!/bin/bash
# Run Nav2 simulation with MoveSequence Pausable Navigator
# Supports pause/resume via /pause topic
#
# After launch, execute:
#   ros2 action send_goal /run_move_sequence_loop solbot4_msgs/action/RunMoveSequenceLoop "{sequence_file: 'long_sequence.json'}"
#
# To pause:
#   ros2 topic pub --once /pause solbot4_msgs/msg/Pause "{paused: true, source: 'operator', reason: 'Manual pause'}"
#
# To resume:
#   ros2 topic pub --once /pause solbot4_msgs/msg/Pause "{paused: false, source: 'operator', reason: 'Resuming'}"

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
USE_MAPVIZ="${USE_MAPVIZ:-False}"

# Get the params file path (pausable version)
PARAMS_FILE=$(ros2 pkg prefix solbot4_nav2_bringup)/share/solbot4_nav2_bringup/params/nav2_params_move_seq_pausable.yaml

echo "=========================================="
echo "  MoveSequence Pausable Navigator"
echo "=========================================="
echo ""
echo "Params file: $PARAMS_FILE"
echo "Headless: $HEADLESS"
echo "Use RViz: $USE_RVIZ"
echo "Use Mapviz: $USE_MAPVIZ"
echo ""
echo "Available actions after launch:"
echo "  - /run_move_sequence_loop (solbot4_msgs/action/RunMoveSequenceLoop)"
echo "  - /move_sequence (solbot4_msgs/action/MoveSequence)"
echo "  - /move (solbot4_msgs/action/Move)"
echo ""
echo "To start sequence:"
echo "  ros2 action send_goal /run_move_sequence_loop solbot4_msgs/action/RunMoveSequenceLoop \"{sequence_file: 'long_sequence.json'}\""
echo ""
echo "To pause:"
echo "  ros2 topic pub --once /pause solbot4_msgs/msg/Pause \"{paused: true, source: 'operator', reason: 'Manual pause'}\""
echo ""
echo "To resume:"
echo "  ros2 topic pub --once /pause solbot4_msgs/msg/Pause \"{paused: false, source: 'operator', reason: 'Resuming'}\""
echo ""
echo "=========================================="

ros2 launch solbot4_nav2_sim nav2_simulation.launch.py \
    headless:=$HEADLESS \
    use_rviz:=$USE_RVIZ \
    use_mapviz:=$USE_MAPVIZ \
    params_file:=$PARAMS_FILE \
    "$@"
