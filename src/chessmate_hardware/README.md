# ChessMate Hardware Interface Package

This ROS 2 package integrates the ChessMate hardware interface components (rotary encoder, LCD display, Arduino communication) with the ROS 2 framework for seamless hardware-software communication.

## Overview

The package provides ROS 2 nodes for:
- **Rotary Encoder Interface**: Publishes user input events from rotary encoder
- **LCD Display Interface**: Manages SSD1306 OLED display for menus and game status
- **Arduino Communication**: Handles serial communication with board sensing and arm control Arduinos
- **Game Management**: Coordinates between chess engine, hardware interfaces, and robot control

## Package Structure

```
chessmate_hardware/
├── chessmate_hardware/
│   ├── __init__.py
│   ├── gpio_abstraction.py          # Hardware abstraction layer
│   ├── rotary_encoder_node.py       # Rotary encoder ROS 2 node
│   ├── lcd_display_node.py          # LCD display ROS 2 node
│   ├── arduino_communication_node.py # Arduino communication ROS 2 node
│   └── game_management_node.py      # Game management coordination node
├── launch/
│   └── hardware_nodes.launch.py    # Launch file for all hardware nodes
├── scripts/
│   ├── test_hardware_integration.py # Integration test script
│   └── demo_hardware_integration.py # Demo script
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

### ROS 2 Dependencies
- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard ROS 2 messages
- `chessmate_msgs` - Custom ChessMate message types

### Python Dependencies
- `pyserial` - Serial communication with Arduino
- `pillow` - Image processing for LCD display
- `RPi.GPIO` - Raspberry Pi GPIO control (optional, for real hardware)

### Hardware Dependencies (Optional)
- SSD1306 OLED display library (for real LCD)
- Arduino IDE and libraries for board/arm controllers

## Installation

1. **Build the package:**
   ```bash
   cd chessmate_dev_ws
   colcon build --packages-select chessmate_hardware --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.10
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Starting All Hardware Nodes

Use the provided launch file to start all hardware interface nodes:

```bash
# For development (mock hardware)
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=mock

# For real hardware (Raspberry Pi)
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real

# Legacy launch file (still available)
ros2 launch chessmate_hardware hardware_nodes.launch.py use_real_hardware:=false
```

### Individual Node Usage

You can also start individual nodes:

```bash
# Rotary encoder node
ros2 run chessmate_hardware rotary_encoder_node

# LCD display node
ros2 run chessmate_hardware lcd_display_node

# Arduino communication node
ros2 run chessmate_hardware arduino_communication_node

# Game management node
ros2 run chessmate_hardware game_management_node
```

### Configuration Parameters

#### Rotary Encoder Node
- `clk_pin`: CLK pin number (default: 11)
- `dt_pin`: DT pin number (default: 12)
- `btn_pin`: Button pin number (default: 13)
- `use_real_gpio`: Use real GPIO hardware (default: auto-detect)
- `publish_rate`: Event publishing rate in Hz (default: 10.0)

#### LCD Display Node
- `use_real_display`: Use real display hardware (default: auto-detect)
- `i2c_bus`: I2C bus number (default: 11)
- `display_width`: Display width in pixels (default: 128)
- `display_height`: Display height in pixels (default: 32)
- `font_path`: Path to TrueType font file
- `small_font_size`: Small font size (default: 8)
- `large_font_size`: Large font size (default: 15)

#### Arduino Communication Node
- `board_controller_port`: Serial port for board controller (default: /dev/ttyUSB0)
- `arm_controller_port`: Serial port for arm controller (default: /dev/ttyUSB1)
- `baud_rate`: Serial communication baud rate (default: 9600)
- `timeout`: Serial timeout in seconds (default: 1.0)
- `use_mock_hardware`: Use mock hardware for development (default: auto-detect)
- `command_timeout`: Command timeout in seconds (default: 5.0)
- `heartbeat_interval`: Heartbeat interval in seconds (default: 10.0)

#### Game Management Node
- `default_skill_level`: Default chess engine skill level (default: 3)
- `default_time_limit`: Default computer thinking time (default: 5.0)
- `menu_timeout`: Menu timeout in seconds (default: 30.0)
- `move_timeout`: Move timeout in seconds (default: 60.0)

## Testing and Demo

### Integration Test
Run the integration test to verify all components work together:

```bash
# Start hardware nodes first
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=mock

# In another terminal, run the test
python3 src/chessmate_hardware/scripts/test_hardware_integration.py
```

### Interactive Demo
Run the interactive demo to see the complete user interaction flow:

```bash
# Start hardware nodes first
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=mock

# In another terminal, run the demo
python3 src/chessmate_hardware/scripts/demo_hardware_integration.py
```

## Hardware Abstraction

The package includes a comprehensive hardware abstraction layer that allows the same code to run on:

1. **Development machines** - Uses mock GPIO and serial interfaces
2. **Raspberry Pi** - Uses real GPIO and hardware interfaces

The abstraction automatically detects the environment and switches between real and mock hardware accordingly.

### Mock Hardware Features
- **Mock GPIO**: Simulates rotary encoder events and button presses
- **Mock Serial**: Simulates Arduino responses for development
- **Mock Display**: Console-based display simulation

## Message Flow

The hardware integration uses the following ROS 2 message flow:

```
Rotary Encoder → RotaryEncoderEvent → Game Management
                                   ↓
LCD Display ← LCDCommand ← Game Management
                                   ↓
Arduino Controllers ← ArduinoCommand ← Game Management
                                   ↓
Board State → Game Management → Chess Engine Services
```

## Integration with Chess Engine

The hardware interface integrates with the existing ChessMate chess engine through ROS 2 services:
- `get_best_move` - Request computer moves
- `evaluate_position` - Position evaluation
- `validate_move` - Move validation
- `update_game_state` - Game state updates

## Troubleshooting

### Common Issues

1. **GPIO Permission Errors (Raspberry Pi)**
   ```bash
   sudo usermod -a -G gpio $USER
   # Log out and back in
   ```

2. **Serial Port Access Errors**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **I2C Display Issues**
   ```bash
   sudo raspi-config
   # Enable I2C in Interface Options
   ```

4. **Python Version Conflicts**
   ```bash
   # Use specific Python version in colcon build
   colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.10
   ```

### Debug Mode

Enable debug logging for detailed information:

```bash
ros2 run chessmate_hardware rotary_encoder_node --ros-args --log-level DEBUG
```

## Future Enhancements

- [ ] Add support for multiple display types
- [ ] Implement advanced menu navigation features
- [ ] Add configuration file support
- [ ] Enhance error recovery mechanisms
- [ ] Add support for additional input devices
- [ ] Implement hardware health monitoring

## Contributing

When contributing to this package:

1. Follow ROS 2 coding standards
2. Maintain hardware abstraction compatibility
3. Add appropriate tests for new features
4. Update documentation for new parameters or features
5. Ensure mock hardware simulation works for development

## License

This package is part of the ChessMate project and follows the same licensing terms.
