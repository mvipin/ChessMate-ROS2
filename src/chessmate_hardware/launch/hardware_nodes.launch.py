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
    # Legacy parameter support
    use_real_hardware_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='false',
        description='[DEPRECATED] Use real hardware (true) or mock hardware (false). Use hardware_mode instead.'
    )

    # New unified parameter
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='mock',
        description='Hardware mode: real, mock, or simulation'
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
    hardware_mode = LaunchConfiguration('hardware_mode')

    # Convert legacy parameter to new format
    def convert_hardware_mode(context):
        use_real = context.launch_configurations.get('use_real_hardware', 'false')
        mode = context.launch_configurations.get('hardware_mode', 'mock')

        # If hardware_mode is explicitly set, use it
        if mode != 'mock':
            return mode
        # Otherwise convert from legacy parameter
        return 'real' if use_real.lower() == 'true' else 'mock'
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
            'hardware_mode': hardware_mode,
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
            'hardware_mode': hardware_mode,
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
            'hardware_mode': hardware_mode,
            'chessboard_controller_port': '/dev/ttyUSB0',
            'robot_controller_port': '/dev/ttyUSB1',
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

    # Robot Animation Controller Node
    robot_animation_controller = Node(
        package='chessmate_hardware',
        executable='robot_animation_controller',
        name='robot_animation_controller',
        output='screen',
        parameters=[{
            'animation_timeout': 30.0,  # Seconds before doze off
            'move_execution_timeout': 60.0,  # Max time for move execution
        }]
    )

    # Group all hardware nodes
    hardware_group = GroupAction([
        rotary_encoder_node,
        lcd_display_node,
        arduino_communication_node,
        game_management_node,
        robot_animation_controller
    ])
    
    return LaunchDescription([
        use_real_hardware_arg,
        hardware_mode_arg,
        skill_level_arg,
        time_limit_arg,
        hardware_group
    ])
