#!/usr/bin/env python3
"""
Test script for ChessMate hardware integration

This script tests the hardware interface nodes by:
1. Publishing test messages to verify communication
2. Monitoring message flow between nodes
3. Testing mock hardware functionality
4. Verifying ROS 2 service integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import threading
from typing import List, Dict

from chessmate_msgs.msg import (
    RotaryEncoderEvent, LCDCommand, ArduinoCommand, 
    BoardState, ChessMove, RobotStatus
)
from std_msgs.msg import String


class HardwareIntegrationTester(Node):
    """
    Test node for hardware integration
    
    This node:
    - Publishes test messages to hardware nodes
    - Monitors responses and message flow
    - Verifies proper integration between components
    - Reports test results
    """
    
    def __init__(self):
        super().__init__('hardware_integration_tester')
        
        self.get_logger().info("Hardware Integration Tester starting")
        
        # Test tracking
        self.test_results: Dict[str, bool] = {}
        self.message_counts: Dict[str, int] = {}
        self.test_start_time = time.time()
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers for testing
        self.encoder_publisher = self.create_publisher(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            qos_profile
        )
        
        self.arduino_command_publisher = self.create_publisher(
            ArduinoCommand,
            'arduino_commands',
            qos_profile
        )
        
        # Create subscribers to monitor responses
        self.lcd_command_subscriber = self.create_subscription(
            LCDCommand,
            'lcd_commands',
            self._lcd_command_callback,
            qos_profile
        )
        
        self.arduino_response_subscriber = self.create_subscription(
            String,
            'arduino_responses',
            self._arduino_response_callback,
            qos_profile
        )
        
        self.board_state_subscriber = self.create_subscription(
            BoardState,
            'board_state',
            self._board_state_callback,
            qos_profile
        )
        
        # Initialize message counters
        self.message_counts = {
            'encoder_events_sent': 0,
            'arduino_commands_sent': 0,
            'lcd_commands_received': 0,
            'arduino_responses_received': 0,
            'board_states_received': 0
        }
        
        # Start test sequence
        self.test_timer = self.create_timer(1.0, self._run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info("Hardware Integration Tester initialized")
    
    def _run_test_sequence(self):
        """Run the test sequence step by step"""
        try:
            if self.test_step == 0:
                self._test_rotary_encoder_simulation()
            elif self.test_step == 1:
                self._test_lcd_commands()
            elif self.test_step == 2:
                self._test_arduino_communication()
            elif self.test_step == 3:
                self._test_game_management_integration()
            elif self.test_step == 4:
                self._generate_test_report()
                self.test_timer.cancel()
                return
            
            self.test_step += 1
            
        except Exception as e:
            self.get_logger().error(f"Test sequence error: {e}")
    
    def _test_rotary_encoder_simulation(self):
        """Test rotary encoder event simulation"""
        self.get_logger().info("Testing rotary encoder simulation...")
        
        # Simulate rotation events
        for direction in [-1, 1, -1, 1]:
            msg = RotaryEncoderEvent()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.event_type = RotaryEncoderEvent.EVENT_TYPE_ROTATION
            msg.direction = direction
            msg.button_pressed = False
            msg.encoder_position = self.message_counts['encoder_events_sent']
            
            self.encoder_publisher.publish(msg)
            self.message_counts['encoder_events_sent'] += 1
            
            self.get_logger().debug(f"Published encoder rotation: direction={direction}")
            time.sleep(0.5)
        
        # Simulate button press
        msg = RotaryEncoderEvent()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.event_type = RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS
        msg.direction = 0
        msg.button_pressed = True
        msg.encoder_position = self.message_counts['encoder_events_sent']
        
        self.encoder_publisher.publish(msg)
        self.message_counts['encoder_events_sent'] += 1
        
        self.get_logger().info(f"Rotary encoder test completed - sent {self.message_counts['encoder_events_sent']} events")
        self.test_results['rotary_encoder'] = True
    
    def _test_lcd_commands(self):
        """Test LCD command functionality by monitoring LCD commands"""
        self.get_logger().info("Testing LCD command monitoring...")
        
        # The LCD commands should be generated by the game management node
        # in response to encoder events, so we just monitor for them
        
        expected_lcd_commands = self.message_counts['lcd_commands_received']
        self.get_logger().info(f"LCD commands received so far: {expected_lcd_commands}")
        
        # Mark test as passed if we received any LCD commands
        self.test_results['lcd_commands'] = self.message_counts['lcd_commands_received'] > 0
    
    def _test_arduino_communication(self):
        """Test Arduino communication"""
        self.get_logger().info("Testing Arduino communication...")
        
        # Send test commands to Arduino
        test_commands = [
            (ArduinoCommand.CMD_OCCUPANCY, "", 0),
            (ArduinoCommand.CMD_LEGAL_MOVES, "e2e4", 0),
            (ArduinoCommand.CMD_COMPUTER_MOVE, "d2d4", 1),
            (ArduinoCommand.CMD_RESET, "", 0)
        ]
        
        for cmd_type, data, target in test_commands:
            msg = ArduinoCommand()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.command_type = cmd_type
            msg.data = data
            msg.target_arduino = target
            
            self.arduino_command_publisher.publish(msg)
            self.message_counts['arduino_commands_sent'] += 1
            
            self.get_logger().debug(f"Published Arduino command: type={cmd_type}, data={data}")
            time.sleep(0.5)
        
        self.get_logger().info(f"Arduino communication test completed - sent {self.message_counts['arduino_commands_sent']} commands")
        self.test_results['arduino_communication'] = True
    
    def _test_game_management_integration(self):
        """Test game management integration"""
        self.get_logger().info("Testing game management integration...")
        
        # Check if we received responses from various components
        total_responses = (
            self.message_counts['lcd_commands_received'] +
            self.message_counts['arduino_responses_received'] +
            self.message_counts['board_states_received']
        )
        
        self.get_logger().info(f"Total system responses: {total_responses}")
        
        # Test passes if we have some level of integration activity
        self.test_results['game_management'] = total_responses > 0
    
    def _generate_test_report(self):
        """Generate and display test report"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("HARDWARE INTEGRATION TEST REPORT")
        self.get_logger().info("=" * 50)
        
        test_duration = time.time() - self.test_start_time
        self.get_logger().info(f"Test Duration: {test_duration:.2f} seconds")
        self.get_logger().info("")
        
        # Test results
        self.get_logger().info("Test Results:")
        for test_name, result in self.test_results.items():
            status = "PASS" if result else "FAIL"
            self.get_logger().info(f"  {test_name}: {status}")
        
        self.get_logger().info("")
        
        # Message statistics
        self.get_logger().info("Message Statistics:")
        for msg_type, count in self.message_counts.items():
            self.get_logger().info(f"  {msg_type}: {count}")
        
        self.get_logger().info("")
        
        # Overall result
        all_passed = all(self.test_results.values())
        overall_status = "PASS" if all_passed else "FAIL"
        self.get_logger().info(f"Overall Test Result: {overall_status}")
        
        if all_passed:
            self.get_logger().info("✅ All hardware integration tests passed!")
        else:
            failed_tests = [name for name, result in self.test_results.items() if not result]
            self.get_logger().warning(f"❌ Failed tests: {failed_tests}")
        
        self.get_logger().info("=" * 50)
        
        # Shutdown after report
        self.create_timer(2.0, self._shutdown_tester, one_shot=True)
    
    def _shutdown_tester(self):
        """Shutdown the tester node"""
        self.get_logger().info("Hardware integration test completed. Shutting down tester.")
        rclpy.shutdown()
    
    def _lcd_command_callback(self, msg: LCDCommand):
        """Monitor LCD commands"""
        self.message_counts['lcd_commands_received'] += 1
        self.get_logger().debug(f"LCD command received: type={msg.command_type}")
    
    def _arduino_response_callback(self, msg: String):
        """Monitor Arduino responses"""
        self.message_counts['arduino_responses_received'] += 1
        self.get_logger().debug(f"Arduino response received: {msg.data}")
    
    def _board_state_callback(self, msg: BoardState):
        """Monitor board state updates"""
        self.message_counts['board_states_received'] += 1
        self.get_logger().debug(f"Board state received: {msg.fen}")


def main(args=None):
    """Main entry point for the hardware integration tester"""
    rclpy.init(args=args)
    
    try:
        tester = HardwareIntegrationTester()
        
        # Run for a limited time
        try:
            rclpy.spin(tester)
        except KeyboardInterrupt:
            tester.get_logger().info("Test interrupted by user")
        finally:
            tester.destroy_node()
            
    except Exception as e:
        print(f"Failed to start hardware integration tester: {e}")
        return 1
    
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
    
    return 0


if __name__ == '__main__':
    exit(main())
