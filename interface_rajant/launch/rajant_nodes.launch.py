#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Rajant interface nodes (query and parser)."""
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='charon',
        description='Name of the robot'
    )

    robot_configs_arg = DeclareLaunchArgument(
        'robot_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'robot_configs.yaml'
        ]),
        description='Path to robot configuration file'
    )

    radio_configs_arg = DeclareLaunchArgument(
        'radio_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'radio_configs.yaml'
        ]),
        description='Path to radio configuration file'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_configs = LaunchConfiguration('robot_configs')
    radio_configs = LaunchConfiguration('radio_configs')

    # Define nodes
    rajant_peer_rssi = Node(
        package='interface_rajant',
        executable='rajant_peer_rssi.py',
        name='rajant_peer_rssi',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'radio_configs': radio_configs
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_configs_arg,
        radio_configs_arg,
        rajant_peer_rssi
    ])
