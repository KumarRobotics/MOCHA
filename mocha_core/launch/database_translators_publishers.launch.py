#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch database, translator, and topic publisher nodes for MOCHA system
    """

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

    topic_configs_arg = DeclareLaunchArgument(
        'topic_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'topic_configs.yaml'
        ]),
        description='Path to topic configuration file'
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
    topic_configs = LaunchConfiguration('topic_configs')
    radio_configs = LaunchConfiguration('radio_configs')

    # Define nodes
    mocha_node = Node(
        package='mocha_core',
        executable='mocha.py',
        name='mocha',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'radio_configs': radio_configs,
            'topic_configs': topic_configs,
            'rssi_threshold': 35
        }]
    )

    translator_node = Node(
        package='mocha_core',
        executable='translator.py',
        name='translator',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs
        }]
    )

    topic_publisher_node = Node(
        package='mocha_core',
        executable='topic_publisher.py',
        name='topic_publisher',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_configs_arg,
        topic_configs_arg,
        radio_configs_arg,
        mocha_node,
        translator_node,
        topic_publisher_node
    ])
