#!/usr/bin/env python3
"""
Topic-Based Arduino Communication Node

This provides the same functionality as arduino_communication_node but uses
topic-based communication instead of ROS2 services to avoid communication issues.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import serial
from chessmate_msgs.msg import ChessMove

class TopicArduinoCommunication(Node):
    def __init__(self):
        super().__init__('topic_arduino_communication')
        
        # Parameters
        self.hardware_mode = self.declare_parameter('hardware_mode', 'mock').value
        self.chessboard_port = self.declare_parameter('chessboard_port', '/dev/ttyACM0').value
        self.robot_port = self.declare_parameter('robot_port', '/dev/ttyACM1').value
        
        # Topic-based service interfaces
        self.setup_topic_services()
        
        # Arduino connections
        self.chessboard_serial = None
        self.robot_serial = None
        self.initialize_connections()
        
        self.get_logger().info("🔧 Topic-based Arduino Communication initialized")
        self.get_logger().info(f"Hardware mode: {self.hardware_mode}")
    
    def setup_topic_services(self):
        """Setup topic-based service interfaces"""
        
        # Robot execute move service (topic-based)
        self.robot_request_subscriber = self.create_subscription(
            String, 'robot/execute_move_request', self.handle_robot_execute_request, 10)
        self.robot_response_publisher = self.create_publisher(
            String, 'robot/execute_move_response', 10)
        
        # Chessboard set mode service (topic-based)
        self.board_request_subscriber = self.create_subscription(
            String, 'chessboard/set_mode_request', self.handle_board_mode_request, 10)
        self.board_response_publisher = self.create_publisher(
            String, 'chessboard/set_mode_response', 10)
        
        # Response tracking
        self.pending_requests = {}
        self.response_lock = threading.Lock()
    
    def initialize_connections(self):
        """Initialize Arduino connections"""
        if self.hardware_mode == 'mock':
            self.get_logger().info("✅ Mock Arduino connections initialized")
        else:
            try:
                # Initialize real serial connections
                self.chessboard_serial = serial.Serial(self.chessboard_port, 9600, timeout=1)
                self.robot_serial = serial.Serial(self.robot_port, 9600, timeout=1)
                self.get_logger().info("✅ Real Arduino connections initialized")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to initialize Arduino connections: {e}")
                self.get_logger().info("Falling back to mock mode")
                self.hardware_mode = 'mock'
    
    def handle_robot_execute_request(self, msg):
        """Handle robot execute move request via topic"""
        try:
            request_data = json.loads(msg.data)
            request_id = request_data.get('id', 'unknown')
            
            self.get_logger().info(f"🤖 Processing robot execute request {request_id}")
            
            # Extract move data
            move_data = request_data.get('move', {})
            from_square = move_data.get('from_square', '')
            to_square = move_data.get('to_square', '')
            piece_type = move_data.get('piece_type', '')
            is_capture = move_data.get('is_capture', False)
            promotion_piece = move_data.get('promotion_piece', '')

            # Execute the move with correct 6-character format
            success = self.execute_robot_move(from_square, to_square, piece_type, is_capture, promotion_piece)
            
            # Prepare response
            response_data = {
                'id': request_id,
                'success': success,
                'message': f"Robot move {'executed' if success else 'failed'}: {from_square} -> {to_square}"
            }
            
            # Publish response
            response_msg = String()
            response_msg.data = json.dumps(response_data)
            self.robot_response_publisher.publish(response_msg)
            
            self.get_logger().info(f"✅ Robot execute response sent for {request_id}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing robot request: {e}")
    
    def handle_board_mode_request(self, msg):
        """Handle chessboard set mode request via topic"""
        try:
            request_data = json.loads(msg.data)
            request_id = request_data.get('id', 'unknown')
            
            self.get_logger().info(f"♟️  Processing board mode request {request_id}")
            
            # Extract mode data
            mode = request_data.get('mode', 2)  # Default to MODE_PLAYING
            
            # Set board mode
            success = self.set_board_mode(mode)
            
            # Prepare response
            response_data = {
                'id': request_id,
                'success': success,
                'message': f"Board mode {'set' if success else 'failed'}: {mode}"
            }
            
            # Publish response
            response_msg = String()
            response_msg.data = json.dumps(response_data)
            self.board_response_publisher.publish(response_msg)
            
            self.get_logger().info(f"✅ Board mode response sent for {request_id}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing board request: {e}")
    
    def execute_robot_move(self, from_square, to_square, piece_type, is_capture=False, promotion_piece=''):
        """Execute robot move (mock or real)"""

        # Format as 6-character move for robot controller
        move_6char = self.format_6char_move(from_square, to_square, piece_type, is_capture, promotion_piece)

        if self.hardware_mode == 'mock':
            # Mock execution with realistic delay
            self.get_logger().info(f"🤖 Mock robot executing 6-char move: {move_6char}")
            self.get_logger().info(f"   ({from_square} -> {to_square}, piece: {piece_type})")
            time.sleep(0.5)  # Simulate robot movement time
            return True
        else:
            try:
                # Send 6-character move command to robot Arduino
                command = f"MOVE:{move_6char}\n"
                self.robot_serial.write(command.encode())

                # Wait for acknowledgment
                response = self.robot_serial.readline().decode().strip()
                success = response.startswith('ACK')

                self.get_logger().info(f"🤖 Robot response: {response}")
                return success

            except Exception as e:
                self.get_logger().error(f"❌ Robot execution failed: {e}")
                return False
    
    def set_board_mode(self, mode):
        """Set chessboard mode (mock or real)"""
        if self.hardware_mode == 'mock':
            # Mock mode setting
            self.get_logger().info(f"♟️  Mock board mode set: {mode}")
            time.sleep(0.1)  # Brief delay
            return True
        else:
            try:
                # Send command to chessboard Arduino
                command = f"MODE:{mode}\n"
                self.chessboard_serial.write(command.encode())
                
                # Wait for acknowledgment
                response = self.chessboard_serial.readline().decode().strip()
                success = response.startswith('ACK')
                
                self.get_logger().info(f"♟️  Board response: {response}")
                return success
                
            except Exception as e:
                self.get_logger().error(f"❌ Board mode setting failed: {e}")
                return False

    def format_6char_move(self, from_square, to_square, piece_type, is_capture=False, promotion_piece=''):
        """Format move as 6-character string for robot controller"""
        # Correct format: from_square(2) + piece(1) + to_square(2) + piece/capture(1)
        # Examples: "e2pe4p", "g1nf3n", "d1qd8x" (capture)

        # Map piece types to single characters (handle both full names and single chars)
        piece_map = {
            'pawn': 'p', 'rook': 'r', 'knight': 'n',
            'bishop': 'b', 'queen': 'q', 'king': 'k',
            'p': 'p', 'r': 'r', 'n': 'n', 'b': 'b', 'q': 'q', 'k': 'k'
        }

        source_piece = piece_map.get(piece_type.lower(), 'p')

        # Destination piece (same as source unless capture or promotion)
        if is_capture:
            dest_piece = 'x'  # Capture indicator
        elif promotion_piece:
            dest_piece = piece_map.get(promotion_piece.lower(), 'q')
        else:
            dest_piece = source_piece  # Same piece

        move_6char = f"{from_square}{source_piece}{to_square}{dest_piece}"
        return move_6char

    def destroy_node(self):
        """Clean up serial connections on shutdown"""
        if self.chessboard_serial:
            try:
                self.chessboard_serial.close()
            except:
                pass
        if self.robot_serial:
            try:
                self.robot_serial.close()
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    
    try:
        arduino_node = TopicArduinoCommunication()
        rclpy.spin(arduino_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            arduino_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
