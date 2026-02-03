#!/bin/bash
# Run Nav2 simulation with Swath Navigator
#
# After launch, execute with coordinates:
#   ros2 action send_goal /run_swath solbot4_msgs/action/RunSwath "{geo_points: [{latitude: 53.5204497, longitude: 17.8259001}, {latitude: 53.5203518, longitude: 17.8258440}], planner_id: 'StraightLine', controller_id: 'FollowPath', path_pub_topic: 'swath_path'}" --feedback

set -e

# Use CycloneDDS for more reliable action server communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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

# Get the params file path
PARAMS_FILE=$(ros2 pkg prefix solbot4_nav2_bringup)/share/solbot4_nav2_bringup/params/nav2_params_swath.yaml

echo "=========================================="
echo "  Swath Navigator"
echo "=========================================="
echo ""
echo "Params file: $PARAMS_FILE"
echo "Headless: $HEADLESS"
echo "Use RViz: $USE_RVIZ"
echo "Use Mapviz: $USE_MAPVIZ"
echo ""
echo "=========================================="

ros2 launch solbot4_nav2_sim nav2_simulation.launch.py \
    headless:=$HEADLESS \
    use_rviz:=$USE_RVIZ \
    use_mapviz:=$USE_MAPVIZ \
    params_file:=$PARAMS_FILE \
    "$@"
