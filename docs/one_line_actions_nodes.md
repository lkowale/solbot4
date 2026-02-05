# Node Diagram: run_one_line_actions.sh

This document shows all nodes launched when running `run_one_line_actions.sh`.

## Launch Hierarchy

```
run_one_line_actions.sh
├── approach_swath_action_node (standalone)
├── run_swath_action_node (standalone)
└── nav2_simulation.launch.py
    ├── simulation.launch.py (Gazebo)
    │   └── spawn_robot.launch.py
    ├── ekf_gps_imu.launch.py (Localization)
    ├── static_transform_publisher (map→odom)
    ├── navigation.launch.py (Nav2)
    └── mapviz (optional)
```

## Node Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                           run_one_line_actions.sh                                       │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                          │
          ┌───────────────────────────────┼───────────────────────────────┐
          │                               │                               │
          ▼                               ▼                               ▼
┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────────────────────┐
│ STANDALONE ACTIONS  │     │ nav2_simulation     │     │                                 │
│ (Started first)     │     │ .launch.py          │     │                                 │
├─────────────────────┤     └──────────┬──────────┘     │                                 │
│ approach_swath_     │                │                │                                 │
│   action_node       │                │                │                                 │
│ run_swath_          │                │                │                                 │
│   action_node       │                │                │                                 │
└─────────────────────┘                │                │                                 │
                                       │                │                                 │
       ┌───────────────┬───────────────┼────────────────┼───────────────┐                 │
       │               │               │                │               │                 │
       ▼               ▼               ▼                ▼               ▼                 │
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐      │
│  GAZEBO SIM  │ │ LOCALIZATION │ │   TF STATIC  │ │ NAVIGATION   │ │   MAPVIZ     │      │
│ simulation   │ │ ekf_gps_imu  │ │              │ │ navigation   │ │ (optional)   │      │
│ .launch.py   │ │ .launch.py   │ │              │ │ .launch.py   │ │              │      │
└──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └──────┬───────┘      │
       │                │                │                │                │              │
       ▼                ▼                ▼                ▼                ▼              │
┌──────────────────────────────────────────────────────────────────────────────────────────┘
```

## All Nodes by Category

### Simulation (simulation.launch.py → spawn_robot.launch.py)

| Node | Package | Description |
|------|---------|-------------|
| gz_sim (server) | ros_gz_sim | Gazebo physics simulation |
| gz_sim (client) | ros_gz_sim | Gazebo GUI (if not headless) |
| robot_state_publisher | robot_state_publisher | Publishes robot URDF to /robot_description |
| bridge_ros_gz | ros_gz_bridge | ROS↔Gazebo topic bridge |
| create | ros_gz_sim | Spawns robot model in Gazebo |
| ackermann_cmd_vel_preprocessor | solbot4_gazebo_spawn | Inverts angular.z when reversing |
| odom_covariance_injector | solbot4_gazebo_spawn | Adds covariance to GPS odometry |
| imu_covariance_injector | solbot4_gazebo_spawn | Adds covariance to IMU data |
| rviz2 | rviz2 | Visualization (optional) |

### Localization (ekf_gps_imu.launch.py)

| Node | Package | Description |
|------|---------|-------------|
| navsat_transform | robot_localization | Converts GPS lat/lon to local odometry |
| ekf_filter_node_odom | robot_localization | Fuses GPS odometry + IMU → /odom |

### Static Transforms

| Node | Package | Description |
|------|---------|-------------|
| static_transform_publisher_map_odom | tf2_ros | map → odom (identity) |
| static_transform_publisher_map_origin | tf2_ros | map → origin (for mapviz) |

### Nav2 Stack (navigation.launch.py)

All nodes run in `nav2_container` (composed mode):

| Node | Package | Description |
|------|---------|-------------|
| controller_server | nav2_controller | Path following (FollowPath, GpsLineFollower) |
| smoother_server | nav2_smoother | Path smoothing |
| planner_server | nav2_planner | Path planning (StraightLine, GridBased) |
| behavior_server | nav2_behaviors | Recovery behaviors (Spin, BackUp, Wait) |
| bt_navigator | nav2_bt_navigator | Behavior tree executor |
| waypoint_follower | nav2_waypoint_follower | Waypoint navigation |
| velocity_smoother | nav2_velocity_smoother | cmd_vel smoothing |
| lifecycle_manager_navigation | nav2_lifecycle_manager | Manages node lifecycles |

### Standalone Action Servers

| Node | Package | Description |
|------|---------|-------------|
| approach_swath_action_node | approach_swath_action | /approach_swath action server |
| run_swath_action_node | run_swath_action | /run_swath action server |

### Visualization (optional)

| Node | Package | Description |
|------|---------|-------------|
| mapviz | mapviz | Map visualization (if USE_MAPVIZ=True) |
| origin_publisher | solbot4_nav2_bringup | WGS84 origin for mapviz |
| rviz2 | rviz2 | 3D visualization (if USE_RVIZ=True) |

## Topic Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              KEY TOPIC FLOW                                             │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│   /gps/fix ──────► navsat_transform ──► /odometry/gps_raw ──► odom_covariance_injector │
│                          │                                            │                │
│   /imu ─────────────────►│                                            ▼                │
│                          │                                     /odometry/gps           │
│                          ▼                                            │                │
│                    ekf_filter_node ◄──────────────────────────────────┘                │
│                          │                                                             │
│                          ▼                                                             │
│                       /odom  (publishes odom→base_footprint TF)                        │
│                                                                                         │
│   bt_navigator ──► controller_server ──► velocity_smoother ──► /cmd_vel                │
│        │                  │                                        │                   │
│        │           /cmd_vel_nav                                    ▼                   │
│        │                                           ackermann_cmd_vel_preprocessor      │
│        │                                                           │                   │
│        ▼                                                           ▼                   │
│   /run_one_line                                           /cmd_vel_ackermann           │
│   (action goal)                                                    │                   │
│                                                                    ▼                   │
│                                                             Gazebo Robot               │
│                                                                                         │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## Summary

**Total nodes: ~20** (varies based on options)

| Category | Count |
|----------|-------|
| Gazebo Simulation | 5-6 |
| Preprocessing | 3 |
| Localization | 2 |
| TF Static | 1-2 |
| Nav2 Stack | 8 |
| Action Servers | 2 |
| Visualization | 0-3 |

## Usage

```bash
# Default (headless, no visualization)
./run_one_line_actions.sh

# With Gazebo GUI
HEADLESS=False ./run_one_line_actions.sh

# With RViz
USE_RVIZ=True ./run_one_line_actions.sh

# With Mapviz
USE_MAPVIZ=True ./run_one_line_actions.sh

# Send goal
ros2 action send_goal /run_one_line solbot4_msgs/action/RunOneLine \
  "{start_point: {latitude: 53.5204497, longitude: 17.8259001}, \
    end_point: {latitude: 53.5203518, longitude: 17.8258440}}" --feedback
```
