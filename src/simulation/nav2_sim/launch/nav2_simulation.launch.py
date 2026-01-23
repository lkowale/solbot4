"""Nav2 simulation launch for solbot4.

Combines:
- Gazebo simulation with robot spawn
- Robot localization (EKF with GPS/IMU fusion)
- Nav2 navigation stack
- Static map->odom transform
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    # Package directories
    gazebo_spawn_dir = get_package_share_directory('solbot4_gazebo_spawn')
    localization_dir = get_package_share_directory('solbot4_localization')
    nav2_bringup_dir = get_package_share_directory('solbot4_nav2_bringup')

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RViz')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start Mapviz')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to run Gazebo in headless mode (no GUI)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start Nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed Nav2 bringup')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params_basic.yaml'),
        description='Nav2 parameters file (use nav2_params_extended.yaml for custom planner)')

    # Gazebo simulation with robot spawn
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_spawn_dir, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Robot localization (EKF with GPS/IMU fusion)
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, 'launch', 'ekf_gps_imu.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Static transform from map to odom (identity transform)
    # Creates fixed relationship between map and odom frames
    static_transform_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom',
                   '--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Nav2 navigation stack
    navigation_cmd = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': use_composition,
                }.items()
            )
        ]
    )

    # Mapviz visualization
    mapviz_config_file = os.path.join(nav2_bringup_dir, 'config', 'mapviz.mvc')
    mapviz_cmd = Node(
        condition=IfCondition(use_mapviz),
        package='mapviz',
        executable='mapviz',
        name='mapviz',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-c', mapviz_config_file],
    )

    # Origin publisher for Mapviz WGS84 transform
    origin_publisher_cmd = Node(
        condition=IfCondition(use_mapviz),
        package='solbot4_nav2_bringup',
        executable='origin_publisher.py',
        name='origin_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Static transform from map to origin frame (for swri_transform_util)
    static_transform_map_to_origin = Node(
        condition=IfCondition(use_mapviz),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_origin',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_params_file_cmd)

    # Launch actions
    ld.add_action(gazebo_cmd)
    ld.add_action(static_transform_map_to_odom)
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(origin_publisher_cmd)
    ld.add_action(static_transform_map_to_origin)

    return ld
