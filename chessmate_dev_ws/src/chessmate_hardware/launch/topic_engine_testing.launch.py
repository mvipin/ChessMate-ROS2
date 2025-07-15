#!/usr/bin/env python3
"""
ChessMate Topic-Based Engine Testing Launch File

This launch file uses the working topic-based chess engine instead of
the broken ROS2 service-based engine.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='mock',
        description='Hardware mode: mock or real'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    
    num_moves_arg = DeclareLaunchArgument(
        'num_moves',
        default_value='5',
        description='Number of moves to test'
    )
    
    # Get launch configuration values
    hardware_mode = LaunchConfiguration('hardware_mode')
    log_level = LaunchConfiguration('log_level')
    num_moves = LaunchConfiguration('num_moves')
    
    # Common parameters
    common_parameters = [
        {'hardware_mode': hardware_mode}
    ]
    
    # Nodes to launch
    nodes = []
    
    # Topic-based Chess Engine Server
    nodes.append(
        Node(
            package='chessmate_engine',
            executable='topic_chess_engine_server',
            name='topic_chess_engine_server',
            parameters=common_parameters,
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        )
    )
    
    # Test End-to-End Game Node (with topic-based engine)
    nodes.append(
        TimerAction(
            period=3.0,  # Wait 3 seconds for engine to start
            actions=[
                Node(
                    package='chessmate_hardware',
                    executable='test_end_to_end_game',
                    name='test_end_to_end_game',
                    parameters=common_parameters + [
                        {'num_moves': num_moves},
                        {'game_timeout': 120}  # 2 minutes
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription([
        hardware_mode_arg,
        log_level_arg,
        num_moves_arg,
    ] + nodes)
