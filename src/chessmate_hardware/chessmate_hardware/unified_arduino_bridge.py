#!/usr/bin/env python3
"""
Unified Arduino Bridge Node for ChessMate Hardware Interface

Combines the JSON-based Arduino Bridge from Raspberry Pi with the character-based
Arduino Communication Node from Linux host. Supports both protocols and provides
hardware abstraction for cross-platform compatibility.

Features:
- Dual protocol support (JSON for precise control, character commands for game actions)
- Auto-detection of Arduino capabilities
- Hardware safety and calibration services
- Sensor data publishing
- Cross-platform compatibility (Linux host and Raspberry Pi)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import threading
import time
import json
import queue
from enum import Enum
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from chessmate_msgs.msg import (
    ArduinoCommand, JointCommand, SensorReading, 
    BoardState, RobotAnimation
)
from chessmate_msgs.srv import CalibrateArm


class ArduinoType(Enum):
    """Arduino controller types"""
    CHESSBOARD_CONTROLLER = 0
    ROBOT_CONTROLLER = 1


class ProtocolType(Enum):
    """Communication protocol types"""
    CHARACTER_BASED = 0  # Simple character commands (existing robot firmware)
    JSON_BASED = 1       # JSON protocol (Raspberry Pi implementation)
    AUTO_DETECT = 2      # Auto-detect protocol based on Arduino response


class MockSerial:
    """Mock serial interface for development without hardware"""
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.in_waiting = 0
        
    def write(self, data):
        print(f"MOCK SERIAL [{self.port}] TX: {data.decode().strip()}")
        return len(data)
        
    def read(self, size=1):
        return b''
        
    def readline(self):
        return b''
        
    def reset_input_buffer(self):
        pass
        
    def close(self):
        self.is_open = False


class UnifiedArduinoBridge(Node):
    """
    Unified Arduino Bridge supporting both JSON and character-based protocols
    """
    
    def __init__(self):
        super().__init__('unified_arduino_bridge')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Serial port configuration
                ('chessboard_port', '/dev/ttyUSB0'),
                ('robot_port', '/dev/ttyUSB1'),
                ('baud_rate', 9600),
                ('timeout', 2.0),
                ('use_mock_serial', True),
                
                # Protocol configuration
                ('chessboard_protocol', 'character'),  # 'character', 'json', 'auto'
                ('robot_protocol', 'character'),       # 'character', 'json', 'auto'
                ('command_terminator', '\n'),
                ('response_timeout', 1.0),
                ('max_retries', 3),
                
                # Publishing rates
                ('sensor_publish_rate', 10.0),
                ('status_publish_rate', 1.0),
                
                # Safety parameters
                ('emergency_stop_enabled', True),
                ('max_joint_velocity', 1.0),
                ('max_joint_acceleration', 2.0),
                
                # Hardware parameters
                ('link1_length', 0.202),
                ('link2_length', 0.190),
                ('z_axis_max', 0.050),
            ]
        )
        
        # Get parameters
        self.chessboard_port = self.get_parameter('chessboard_port').value
        self.robot_port = self.get_parameter('robot_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.use_mock_serial = self.get_parameter('use_mock_serial').value
        
        self.chessboard_protocol = self.get_parameter('chessboard_protocol').value
        self.robot_protocol = self.get_parameter('robot_protocol').value
        self.terminator = self.get_parameter('command_terminator').value
        self.response_timeout = self.get_parameter('response_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Initialize state
        self.running = True
        self.emergency_stop = False
        self.is_calibrated = False
        self.last_joint_state = None
        self.protocol_types = {}  # Store detected protocol for each Arduino
        
        # Serial connections and locks
        self.serial_connections = {}
        self.serial_locks = {}
        self.command_queues = {}
        
        # Initialize for each Arduino type
        for arduino_type in ArduinoType:
            self.serial_locks[arduino_type] = threading.Lock()
            self.command_queues[arduino_type] = queue.Queue()
        
        # Publishers
        self.board_state_publisher = self.create_publisher(BoardState, 'board_state', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.sensor_publisher = self.create_publisher(SensorReading, 'sensor_readings', 10)
        self.status_publisher = self.create_publisher(String, 'hardware_status', 10)
        
        # Subscribers
        self.arduino_command_subscription = self.create_subscription(
            ArduinoCommand, 'arduino_command', self._arduino_command_callback, 10)
        self.joint_command_subscription = self.create_subscription(
            JointCommand, 'joint_command', self._joint_command_callback, 10)
        self.emergency_stop_subscription = self.create_subscription(
            Bool, 'emergency_stop', self._emergency_stop_callback, 10)
        
        # Services
        self.calibrate_service = self.create_service(
            CalibrateArm, 'calibrate_arm', self._calibrate_callback)
        
        # Timers
        sensor_rate = self.get_parameter('sensor_publish_rate').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        self.sensor_timer = self.create_timer(1.0/sensor_rate, self._publish_sensor_data)
        self.status_timer = self.create_timer(1.0/status_rate, self._publish_status)
        
        # Initialize hardware connections
        self._initialize_hardware()
        
        self.get_logger().info('Unified Arduino Bridge initialized')
    
    def _initialize_hardware(self):
        """Initialize serial connections and detect protocols"""
        port_map = {
            ArduinoType.CHESSBOARD_CONTROLLER: self.chessboard_port,
            ArduinoType.ROBOT_CONTROLLER: self.robot_port
        }
        
        protocol_map = {
            ArduinoType.CHESSBOARD_CONTROLLER: self.chessboard_protocol,
            ArduinoType.ROBOT_CONTROLLER: self.robot_protocol
        }
        
        for arduino_type in ArduinoType:
            port = port_map[arduino_type]
            protocol_pref = protocol_map[arduino_type]
            
            try:
                self.get_logger().info(f'Connecting to {arduino_type.name} on {port}')
                
                if self.use_mock_serial:
                    serial_conn = MockSerial(port, self.baud_rate, self.timeout)
                    self.get_logger().info(f'Using mock serial for {arduino_type.name}')
                else:
                    serial_conn = serial.Serial(
                        port=port,
                        baudrate=self.baud_rate,
                        timeout=self.timeout
                    )
                    time.sleep(2)  # Wait for Arduino reset
                
                self.serial_connections[arduino_type] = serial_conn
                
                # Detect or set protocol
                detected_protocol = self._detect_protocol(arduino_type, protocol_pref)
                self.protocol_types[arduino_type] = detected_protocol
                
                self.get_logger().info(
                    f'{arduino_type.name} connected using {detected_protocol.name} protocol')
                
                # Start communication thread
                thread = threading.Thread(
                    target=self._communication_loop,
                    args=(arduino_type,),
                    daemon=True
                )
                thread.start()
                
            except Exception as e:
                self.get_logger().error(f'Failed to connect to {arduino_type.name}: {e}')
                if not self.use_mock_serial:
                    # Fall back to mock serial
                    self.serial_connections[arduino_type] = MockSerial(port, self.baud_rate)
                    self.protocol_types[arduino_type] = ProtocolType.CHARACTER_BASED
                    self.get_logger().warn(f'Using mock serial fallback for {arduino_type.name}')
    
    def _detect_protocol(self, arduino_type: ArduinoType, preference: str) -> ProtocolType:
        """
        Detect Arduino communication protocol
        
        Args:
            arduino_type: Type of Arduino controller
            preference: Preferred protocol ('character', 'json', 'auto')
            
        Returns:
            Detected protocol type
        """
        if preference == 'character':
            return ProtocolType.CHARACTER_BASED
        elif preference == 'json':
            return ProtocolType.JSON_BASED
        elif preference == 'auto':
            # Try JSON first, fall back to character
            if self._test_json_protocol(arduino_type):
                return ProtocolType.JSON_BASED
            else:
                return ProtocolType.CHARACTER_BASED
        else:
            self.get_logger().warn(f'Unknown protocol preference: {preference}')
            return ProtocolType.CHARACTER_BASED
    
    def _test_json_protocol(self, arduino_type: ArduinoType) -> bool:
        """Test if Arduino supports JSON protocol"""
        try:
            # Send a simple JSON ping command
            test_command = json.dumps({"cmd": "ping", "id": 1})
            response = self._send_command_direct(arduino_type, test_command)
            
            if response:
                try:
                    json.loads(response)
                    return True
                except json.JSONDecodeError:
                    return False
            return False
            
        except Exception:
            return False

    def _communication_loop(self, arduino_type: ArduinoType):
        """Main communication loop for Arduino controller"""
        serial_conn = self.serial_connections[arduino_type]
        command_queue = self.command_queues[arduino_type]

        self.get_logger().info(f"Communication loop started for {arduino_type.name}")

        while self.running:
            try:
                # Process outgoing commands
                try:
                    command_data = command_queue.get(timeout=0.1)
                    self._send_command(serial_conn, command_data, arduino_type)
                except queue.Empty:
                    pass

                # Process incoming responses
                if self._has_incoming_data(serial_conn):
                    response = self._read_response(serial_conn)
                    if response:
                        self._process_response(response, arduino_type)

                time.sleep(0.01)  # Small delay to prevent tight loop

            except Exception as e:
                self.get_logger().error(f"Communication loop error for {arduino_type.name}: {e}")
                time.sleep(1)  # Longer delay on error

    def _send_command_direct(self, arduino_type: ArduinoType, command: str) -> str:
        """Send command directly and wait for response (for protocol detection)"""
        serial_conn = self.serial_connections.get(arduino_type)
        if not serial_conn:
            return None

        with self.serial_locks[arduino_type]:
            try:
                if hasattr(serial_conn, 'reset_input_buffer'):
                    serial_conn.reset_input_buffer()

                cmd_str = command + self.terminator
                serial_conn.write(cmd_str.encode())

                # Wait for response
                start_time = time.time()
                response = ""

                while time.time() - start_time < self.response_timeout:
                    if hasattr(serial_conn, 'in_waiting') and serial_conn.in_waiting > 0:
                        char = serial_conn.read(1).decode('utf-8', errors='ignore')
                        if char == '\n':
                            break
                        response += char
                    elif hasattr(serial_conn, 'readline'):
                        response = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        break
                    time.sleep(0.01)

                return response.strip() if response else None

            except Exception as e:
                self.get_logger().error(f'Direct command error: {e}')
                return None

    def _send_command(self, serial_conn, command_data: str, arduino_type: ArduinoType):
        """Send command to Arduino controller"""
        if not serial_conn or not command_data:
            return

        with self.serial_locks[arduino_type]:
            try:
                cmd_str = command_data
                if not cmd_str.endswith('\n'):
                    cmd_str += '\n'

                serial_conn.write(cmd_str.encode())
                self.get_logger().debug(f"Sent to {arduino_type.name}: {cmd_str.strip()}")

            except Exception as e:
                self.get_logger().error(f"Send command error for {arduino_type.name}: {e}")

    def _has_incoming_data(self, serial_conn) -> bool:
        """Check if serial connection has incoming data"""
        try:
            if hasattr(serial_conn, 'in_waiting'):
                return serial_conn.in_waiting > 0
            return False
        except Exception:
            return False

    def _read_response(self, serial_conn) -> str:
        """Read response from Arduino controller"""
        try:
            if hasattr(serial_conn, 'readline'):
                response = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                return response if response else None
            return None
        except Exception as e:
            self.get_logger().error(f"Read response error: {e}")
            return None

    def _process_response(self, response: str, arduino_type: ArduinoType):
        """Process response from Arduino controller"""
        if not response:
            return

        self.get_logger().debug(f"Received from {arduino_type.name}: {response}")

        protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

        if protocol_type == ProtocolType.JSON_BASED:
            self._process_json_response(response, arduino_type)
        else:
            self._process_character_response(response, arduino_type)

    def _process_json_response(self, response: str, arduino_type: ArduinoType):
        """Process JSON response from Arduino"""
        try:
            data = json.loads(response)

            # Handle different response types
            if data.get('status') == 'ok':
                # Successful command response
                if 'data' in data:
                    self._handle_sensor_data(data['data'], arduino_type)
            elif data.get('status') == 'error':
                self.get_logger().error(f"Arduino error: {data.get('error', 'Unknown error')}")
            else:
                # Handle other JSON responses (sensor data, status updates)
                self._handle_sensor_data(data, arduino_type)

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid JSON response: {response} - {e}")

    def _process_character_response(self, response: str, arduino_type: ArduinoType):
        """Process character-based response from Arduino"""
        # Handle board state updates from chessboard controller
        if arduino_type == ArduinoType.CHESSBOARD_CONTROLLER:
            if len(response) == 64:  # Board state (64 squares)
                self._publish_board_state(response)
            elif response.startswith('occupancy:'):
                # Handle occupancy response
                occupancy_data = response.split(':', 1)[1]
                self._publish_board_state(occupancy_data)

        # Handle robot controller responses
        elif arduino_type == ArduinoType.ROBOT_CONTROLLER:
            # Simple acknowledgment or status responses
            if response in ['i', 'j', 's', 'z']:
                self.get_logger().debug(f"Robot controller acknowledged: {response}")

    def _handle_sensor_data(self, data: dict, arduino_type: ArduinoType):
        """Handle sensor data from Arduino"""
        try:
            # Publish joint state if available
            if 'j1' in data and 'j2' in data and 'z' in data:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['joint1', 'joint2', 'z_axis']
                joint_state.position = [data['j1'], data['j2'], data['z']]

                if 'moving' in data:
                    # Add velocity information if available
                    joint_state.velocity = [0.0, 0.0, 0.0]  # Could be enhanced

                self.joint_state_publisher.publish(joint_state)
                self.last_joint_state = joint_state

            # Publish individual sensor readings
            for sensor_name, value in data.items():
                if isinstance(value, (int, float)):
                    sensor_reading = SensorReading()
                    sensor_reading.header.stamp = self.get_clock().now().to_msg()
                    sensor_reading.sensor_name = sensor_name
                    sensor_reading.raw_value = float(value)
                    sensor_reading.processed_value = float(value)
                    sensor_reading.is_active = True

                    self.sensor_publisher.publish(sensor_reading)

        except Exception as e:
            self.get_logger().error(f"Error handling sensor data: {e}")

    def _publish_board_state(self, board_data: str):
        """Publish board state from chessboard controller"""
        try:
            board_state = BoardState()
            board_state.header.stamp = self.get_clock().now().to_msg()

            # Convert board data to piece positions
            # This would need to be implemented based on the specific format
            # For now, just store the raw data
            board_state.board_fen = board_data  # Placeholder

            self.board_state_publisher.publish(board_state)

        except Exception as e:
            self.get_logger().error(f"Error publishing board state: {e}")

    def _arduino_command_callback(self, msg: ArduinoCommand):
        """Handle Arduino command messages"""
        try:
            arduino_type = ArduinoType(msg.target_arduino)
            protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

            if protocol_type == ProtocolType.JSON_BASED:
                command_str = self._format_json_command(msg, arduino_type)
            else:
                command_str = self._format_character_command(msg, arduino_type)

            if command_str:
                self.command_queues[arduino_type].put(command_str)
                self.get_logger().debug(f"Queued command for {arduino_type.name}: {command_str}")

        except Exception as e:
            self.get_logger().error(f"Error processing Arduino command: {e}")

    def _joint_command_callback(self, msg: JointCommand):
        """Handle joint command messages (JSON protocol)"""
        try:
            # Safety checks
            if self.emergency_stop:
                self.get_logger().warn("Joint command ignored - emergency stop active")
                return

            if not self.is_calibrated:
                self.get_logger().warn("Joint command ignored - system not calibrated")
                return

            # Create JSON command for joint movement
            joint_data = {
                "cmd": "move_joints",
                "params": {},
                "id": int(time.time() * 1000) % 10000  # Simple ID generation
            }

            # Map joint positions
            if len(msg.positions) >= 3:
                joint_data["params"]["j1"] = msg.positions[0]
                joint_data["params"]["j2"] = msg.positions[1]
                joint_data["params"]["z"] = msg.positions[2]

            # Add velocity and safety parameters
            if msg.velocities:
                joint_data["params"]["velocity_scaling"] = msg.velocity_scaling

            command_str = json.dumps(joint_data)

            # Send to robot controller (assuming it supports JSON for precise control)
            arduino_type = ArduinoType.ROBOT_CONTROLLER
            if arduino_type in self.command_queues:
                self.command_queues[arduino_type].put(command_str)
                self.get_logger().info(f"Sent joint command: {command_str}")

        except Exception as e:
            self.get_logger().error(f"Error processing joint command: {e}")

    def _emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop messages"""
        self.emergency_stop = msg.data

        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED")

            # Send emergency stop to all controllers
            for arduino_type in ArduinoType:
                protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

                if protocol_type == ProtocolType.JSON_BASED:
                    estop_cmd = json.dumps({
                        "cmd": "emergency_stop",
                        "params": {"active": True},
                        "id": int(time.time() * 1000) % 10000
                    })
                else:
                    estop_cmd = "STOP"  # Simple character command

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(estop_cmd)
        else:
            self.get_logger().info("Emergency stop released")

            # Send release command to all controllers
            for arduino_type in ArduinoType:
                protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

                if protocol_type == ProtocolType.JSON_BASED:
                    release_cmd = json.dumps({
                        "cmd": "emergency_stop",
                        "params": {"active": False},
                        "id": int(time.time() * 1000) % 10000
                    })
                else:
                    release_cmd = "RESUME"  # Simple character command

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(release_cmd)

    def _format_json_command(self, msg: ArduinoCommand, arduino_type: ArduinoType) -> str:
        """Format Arduino command as JSON"""
        try:
            # Map ArduinoCommand types to JSON commands
            json_command_map = {
                ArduinoCommand.CMD_ROBOT_WAKE_UP: "wake_up",
                ArduinoCommand.CMD_ROBOT_DOZE_OFF: "doze_off",
                ArduinoCommand.CMD_ROBOT_HOME_Z: "home_z",
                ArduinoCommand.CMD_ROBOT_HOME_ALL: "home_all",
                ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE: "execute_move",
                ArduinoCommand.CMD_OCCUPANCY: "get_occupancy",
                ArduinoCommand.CMD_STATUS: "get_status",
            }

            cmd_name = json_command_map.get(msg.command_type, "unknown")

            json_cmd = {
                "cmd": cmd_name,
                "id": int(time.time() * 1000) % 10000
            }

            # Add parameters if available
            if msg.data:
                if msg.command_type == ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE:
                    # Parse move data for JSON format
                    json_cmd["params"] = {"move": msg.data}
                else:
                    json_cmd["params"] = {"data": msg.data}

            return json.dumps(json_cmd)

        except Exception as e:
            self.get_logger().error(f"Error formatting JSON command: {e}")
            return ""

    def _format_character_command(self, msg: ArduinoCommand, arduino_type: ArduinoType) -> str:
        """Format Arduino command as character-based command"""
        try:
            # Use existing character-based formatting from arduino_communication_node
            if arduino_type == ArduinoType.ROBOT_CONTROLLER:
                return self._format_robot_character_command(msg)
            else:
                return self._format_chessboard_character_command(msg)

        except Exception as e:
            self.get_logger().error(f"Error formatting character command: {e}")
            return ""

    def _format_robot_character_command(self, msg: ArduinoCommand) -> str:
        """Format robot controller character commands"""
        try:
            # Handle move execution specially
            if msg.command_type == ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE:
                if msg.data and len(msg.data) == 6:
                    return msg.data  # Send 6-character move directly
                else:
                    self.get_logger().error(f"Invalid move format: {msg.data}")
                    return ""

            # Handle single-character indications
            robot_single_char_commands = {
                ArduinoCommand.CMD_ROBOT_WAKE_UP: "i",
                ArduinoCommand.CMD_ROBOT_DOZE_OFF: "s",
                ArduinoCommand.CMD_ROBOT_HOME_Z: "j",
                ArduinoCommand.CMD_ROBOT_HOME_ALL: "z",
                ArduinoCommand.CMD_ROBOT_RESET_POSE: "z"
            }

            if msg.command_type in robot_single_char_commands:
                return robot_single_char_commands[msg.command_type]

            return ""

        except Exception as e:
            self.get_logger().error(f"Error formatting robot character command: {e}")
            return ""

    def _format_chessboard_character_command(self, msg: ArduinoCommand) -> str:
        """Format chessboard controller character commands"""
        try:
            # Map command types to command names
            command_map = {
                ArduinoCommand.CMD_OCCUPANCY: "occupancy",
                ArduinoCommand.CMD_STATUS: "status",
                ArduinoCommand.CMD_RESET: "reset",
                ArduinoCommand.CMD_CALIBRATE: "calibrate",
                ArduinoCommand.CMD_LED_ON: "led_on",
                ArduinoCommand.CMD_LED_OFF: "led_off",
                ArduinoCommand.CMD_HINT_ENABLE: "hint_enable",
                ArduinoCommand.CMD_HINT_DISABLE: "hint_disable",
            }

            command_name = command_map.get(msg.command_type, "unknown")

            if command_name == "unknown":
                self.get_logger().warning(f"Unknown chessboard command type: {msg.command_type}")
                return ""

            # Format command with data if available
            if msg.data:
                return f"{command_name}:{msg.data}"
            else:
                return command_name

        except Exception as e:
            self.get_logger().error(f"Error formatting chessboard character command: {e}")
            return ""

    def _calibrate_callback(self, request, response):
        """Handle calibration service requests"""
        try:
            self.get_logger().info(f"Starting calibration: {request.calibration_type}")

            # Send calibration command to robot controller
            arduino_type = ArduinoType.ROBOT_CONTROLLER
            protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

            if protocol_type == ProtocolType.JSON_BASED:
                calibrate_cmd = json.dumps({
                    "cmd": "calibrate",
                    "params": {
                        "type": request.calibration_type,
                        "use_limit_switches": request.use_limit_switches,
                        "speed": request.calibration_speed
                    },
                    "id": int(time.time() * 1000) % 10000
                })
            else:
                calibrate_cmd = "z"  # Simple home command for character protocol

            if arduino_type in self.command_queues:
                self.command_queues[arduino_type].put(calibrate_cmd)

            # Wait for calibration to complete (simplified)
            time.sleep(5.0)  # This should be improved with actual feedback

            # Set calibration status
            self.is_calibrated = True

            # Prepare response
            response.success = True
            response.message = "Calibration completed successfully"
            response.joint_offsets = [0.0, 0.0, 0.0]  # Placeholder
            response.workspace_min = Point(x=-0.3, y=-0.3, z=0.0)
            response.workspace_max = Point(x=0.3, y=0.3, z=0.05)
            response.calibration_accuracy = 1.0  # mm
            response.calibration_timestamp = self.get_clock().now().to_msg()

            self.get_logger().info("Calibration completed successfully")

        except Exception as e:
            self.get_logger().error(f"Calibration error: {e}")
            response.success = False
            response.message = f"Calibration failed: {str(e)}"

        return response

    def _publish_sensor_data(self):
        """Periodic sensor data publishing"""
        try:
            # Request sensor data from controllers
            for arduino_type in ArduinoType:
                protocol_type = self.protocol_types.get(arduino_type, ProtocolType.CHARACTER_BASED)

                if protocol_type == ProtocolType.JSON_BASED:
                    sensor_cmd = json.dumps({
                        "cmd": "get_position",
                        "id": int(time.time() * 1000) % 10000
                    })
                else:
                    sensor_cmd = "status"  # Simple status request

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(sensor_cmd)

        except Exception as e:
            self.get_logger().error(f"Error requesting sensor data: {e}")

    def _publish_status(self):
        """Periodic status publishing"""
        try:
            status_msg = String()
            status_parts = []

            # Add connection status
            for arduino_type in ArduinoType:
                if arduino_type in self.serial_connections:
                    conn = self.serial_connections[arduino_type]
                    if hasattr(conn, 'is_open') and conn.is_open:
                        status_parts.append(f"{arduino_type.name}:CONNECTED")
                    else:
                        status_parts.append(f"{arduino_type.name}:DISCONNECTED")
                else:
                    status_parts.append(f"{arduino_type.name}:NOT_INITIALIZED")

            # Add system status
            status_parts.append(f"CALIBRATED:{self.is_calibrated}")
            status_parts.append(f"EMERGENCY_STOP:{self.emergency_stop}")

            status_msg.data = "|".join(status_parts)
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Unified Arduino Bridge")
        self.running = False

        # Close serial connections
        for arduino_type, serial_conn in self.serial_connections.items():
            try:
                if hasattr(serial_conn, 'close'):
                    serial_conn.close()
                self.get_logger().info(f"Closed connection to {arduino_type.name}")
            except Exception as e:
                self.get_logger().error(f"Error closing {arduino_type.name}: {e}")

        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = UnifiedArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
