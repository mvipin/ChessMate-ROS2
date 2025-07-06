#!/usr/bin/env python3
"""
Simple Robot Communication Test

This script tests the direct host-to-robot communication by sending
basic robot commands and verifying they reach the robot controller.
"""

import rclpy
from rclpy.node import Node
from chessmate_msgs.msg import ArduinoCommand
from chessmate_hardware.arduino_communication_node import ArduinoType
from std_msgs.msg import String
import time


class SimpleRobotTester(Node):
    """Simple test node for robot communication"""
    
    def __init__(self):
        super().__init__('simple_robot_tester')
        
        self.get_logger().info("Simple Robot Tester starting...")
        
        # Publisher for Arduino commands
        self.arduino_command_publisher = self.create_publisher(
            ArduinoCommand,
            'arduino_command',
            10
        )
        
        # Subscriber to monitor Arduino responses
        self.arduino_response_subscriber = self.create_subscription(
            String,
            'arduino_responses',
            self.arduino_response_callback,
            10
        )
        
        self.responses_received = []
        
        # Wait for connections
        time.sleep(2.0)
        
        self.get_logger().info("Simple Robot Tester initialized")
    
    def arduino_response_callback(self, msg):
        """Monitor Arduino responses"""
        self.responses_received.append(msg.data)
        self.get_logger().info(f"Arduino response: {msg.data}")
    
    def send_robot_command(self, command_type, description, data=""):
        """Send a robot command"""
        self.get_logger().info(f"Sending: {description}")
        
        cmd = ArduinoCommand()
        cmd.timestamp = self.get_clock().now().to_msg()
        cmd.command_type = command_type
        cmd.target_arduino = ArduinoType.ROBOT_CONTROLLER.value
        cmd.data = data
        
        self.arduino_command_publisher.publish(cmd)
        self.get_logger().info(f"Command sent: type={command_type}, target=robot_controller, data='{data}'")
    
    def run_basic_tests(self):
        """Run basic robot communication tests"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("BASIC ROBOT COMMUNICATION TESTS")
        self.get_logger().info("=" * 50)
        
        # Test 1: Wake up animation
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_WAKE_UP,
            "Robot Wake Up Animation (should send 'i')"
        )
        time.sleep(2.0)
        
        # Test 2: Think hard animation  
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_THINK_HARD,
            "Robot Think Hard Animation (should send 'i')"
        )
        time.sleep(2.0)
        
        # Test 3: Home Z position
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_HOME_Z,
            "Robot Home Z Position (should send 'j')"
        )
        time.sleep(2.0)
        
        # Test 4: Home all positions
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_HOME_ALL,
            "Robot Home All Positions (should send 'z')"
        )
        time.sleep(2.0)
        
        # Test 5: Reset pose
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_RESET_POSE,
            "Robot Reset Pose (should send 'z')"
        )
        time.sleep(2.0)
        
        # Test 6: Doze off animation
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_DOZE_OFF,
            "Robot Doze Off Animation (should send 's')"
        )
        time.sleep(2.0)
        
        # Test 7: Execute move
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE,
            "Robot Execute Move (should send 'e2pe4p')",
            "e2pe4p"
        )
        time.sleep(3.0)
        
        # Test 8: Execute capture move
        self.send_robot_command(
            ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE,
            "Robot Execute Capture (should send 'e4pf7x')",
            "e4pf7x"
        )
        time.sleep(3.0)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("TESTS COMPLETED")
        self.get_logger().info("=" * 50)
        
        # Summary
        self.get_logger().info(f"Total commands sent: 8")
        self.get_logger().info(f"Arduino responses received: {len(self.responses_received)}")
        
        if self.responses_received:
            self.get_logger().info("Responses:")
            for i, response in enumerate(self.responses_received):
                self.get_logger().info(f"  {i+1}: {response}")
        else:
            self.get_logger().warning("No responses received - check if arduino_communication_node is running")
        
        self.get_logger().info("\nExpected robot controller serial output:")
        self.get_logger().info("  i  (wake up)")
        self.get_logger().info("  i  (think hard)")  
        self.get_logger().info("  j  (home Z)")
        self.get_logger().info("  z  (home all)")
        self.get_logger().info("  z  (reset pose)")
        self.get_logger().info("  s  (doze off)")
        self.get_logger().info("  e2pe4p  (execute move)")
        self.get_logger().info("  e4pf7x  (execute capture)")


def main():
    """Main function"""
    rclpy.init()
    
    try:
        tester = SimpleRobotTester()
        
        # Give some time for node to initialize
        time.sleep(1.0)
        
        # Run the tests
        tester.run_basic_tests()
        
        # Keep node alive for a bit to receive any delayed responses
        tester.get_logger().info("Waiting for any delayed responses...")
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        try:
            tester.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
