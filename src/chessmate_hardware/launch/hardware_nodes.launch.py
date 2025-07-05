#!/usr/bin/env python3
"""
Launch file for ChessMate hardware interface nodes

This launch file starts all the hardware interface nodes:
- Rotary encoder node
- LCD display node  
- Arduino communication node
- Game management node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for hardware nodes"""
    
    # Declare launch arguments
    use_real_hardware_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='false',
        description='Use real hardware (true) or mock hardware (false)'
    )
    
    skill_level_arg = DeclareLaunchArgument(
        'skill_level',
        default_value='3',
        description='Default chess engine skill level (0-20)'
    )
    
    time_limit_arg = DeclareLaunchArgument(
        'time_limit',
        default_value='5.0',
        description='Default computer thinking time in seconds'
    )
    
    # Get launch configuration values
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    skill_level = LaunchConfiguration('skill_level')
    time_limit = LaunchConfiguration('time_limit')
    
    # Rotary encoder node
    rotary_encoder_node = Node(
        package='chessmate_hardware',
        executable='rotary_encoder_node',
        name='rotary_encoder_node',
        parameters=[{
            'clk_pin': 11,
            'dt_pin': 12,
            'btn_pin': 13,
            'use_real_gpio': use_real_hardware,
            'publish_rate': 10.0
        }],
        output='screen'
    )
    
    # LCD display node
    lcd_display_node = Node(
        package='chessmate_hardware',
        executable='lcd_display_node',
        name='lcd_display_node',
        parameters=[{
            'use_real_display': use_real_hardware,
            'i2c_bus': 11,
            'display_width': 128,
            'display_height': 32,
            'font_path': '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
            'small_font_size': 8,
            'large_font_size': 15
        }],
        output='screen'
    )
    
    # Arduino communication node
    arduino_communication_node = Node(
        package='chessmate_hardware',
        executable='arduino_communication_node',
        name='arduino_communication_node',
        parameters=[{
            'board_controller_port': '/dev/ttyUSB0',
            'arm_controller_port': '/dev/ttyUSB1',
            'baud_rate': 9600,
            'timeout': 1.0,
            'command_timeout': 5.0,
            'heartbeat_interval': 10.0
        }],
        output='screen'
    )
    
    # Game management node
    game_management_node = Node(
        package='chessmate_hardware',
        executable='game_management_node',
        name='game_management_node',
        parameters=[{
            'default_skill_level': skill_level,
            'default_time_limit': time_limit,
            'menu_timeout': 30.0,
            'move_timeout': 60.0
        }],
        output='screen'
    )
    
    # Group all hardware nodes
    hardware_group = GroupAction([
        rotary_encoder_node,
        lcd_display_node,
        arduino_communication_node,
        game_management_node
    ])
    
    return LaunchDescription([
        use_real_hardware_arg,
        skill_level_arg,
        time_limit_arg,
        hardware_group
    ])
