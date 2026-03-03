#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch titan robot with database, translators, publishers, and Rajant interface
    """
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='titan',
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
    
    # Include database, translators and publishers launch file
    database_translators_publishers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mocha_core'),
                'launch',
                'database_translators_publishers.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs,
            'radio_configs': radio_configs
        }.items()
    )
    
    # Include Rajant interface launch file
    rajant_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interface_rajant'),
                'launch',
                'rajant_nodes.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs,
            'radio_configs': radio_configs
        }.items()
    )
    
    return LaunchDescription([
        robot_name_arg,
        robot_configs_arg,
        topic_configs_arg,
        radio_configs_arg,
        database_translators_publishers_launch,
        rajant_interface_launch
    ])