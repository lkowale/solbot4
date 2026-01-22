"""Spawn solbot4 robot in Gazebo and set up ROS-Gazebo bridges."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('solbot4_gazebo_spawn')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_simulator = LaunchConfiguration('use_simulator')
    robot_name = LaunchConfiguration('robot_name')

    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00')
    }

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='solbot4',
        description='Name of the robot'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    sim_dir, 'configs', 'solbot4_gz_bridge.yaml'
                ),
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    # Spawn robot model
    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-R', pose['R'],
            '-P', pose['P'],
            '-Y', pose['Y']
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(bridge)
    ld.add_action(spawn_model)

    return ld
