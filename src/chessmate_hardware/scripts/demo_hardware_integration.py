#!/usr/bin/env python3
"""
Demo script for ChessMate hardware integration

This script demonstrates the complete hardware integration by:
1. Starting all hardware nodes
2. Simulating user interactions
3. Showing message flow between components
4. Demonstrating game management integration
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


class HardwareIntegrationDemo(Node):
    """
    Demo node for hardware integration
    
    This node demonstrates:
    - User interface interactions (rotary encoder + LCD)
    - Arduino communication for board sensing and arm control
    - Game management coordination
    - Integration with chess engine services
    """
    
    def __init__(self):
        super().__init__('hardware_integration_demo')
        
        self.get_logger().info("🎮 ChessMate Hardware Integration Demo Starting")
        
        # Demo state
        self.demo_step = 0
        self.demo_running = True
        self.message_log: List[str] = []
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers for demo
        self.encoder_publisher = self.create_publisher(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            qos_profile
        )
        
        # Create subscribers to monitor all system activity
        self.lcd_command_subscriber = self.create_subscription(
            LCDCommand,
            'lcd_commands',
            self._lcd_command_callback,
            qos_profile
        )
        
        self.arduino_command_subscriber = self.create_subscription(
            ArduinoCommand,
            'arduino_commands',
            self._arduino_command_callback,
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
        
        # Start demo sequence
        self.get_logger().info("📋 Demo will simulate complete user interaction flow")
        self.get_logger().info("🔄 Starting demo sequence in 3 seconds...")
        
        self.demo_timer = self.create_timer(3.0, self._start_demo_sequence, one_shot=True)
    
    def _start_demo_sequence(self):
        """Start the demo sequence"""
        self.get_logger().info("🚀 Demo sequence starting!")
        self.demo_timer = self.create_timer(2.0, self._run_demo_step)
    
    def _run_demo_step(self):
        """Run each step of the demo"""
        try:
            if self.demo_step == 0:
                self._demo_startup_sequence()
            elif self.demo_step == 1:
                self._demo_menu_navigation()
            elif self.demo_step == 2:
                self._demo_game_setup()
            elif self.demo_step == 3:
                self._demo_new_game_selection()
            elif self.demo_step == 4:
                self._demo_game_start()
            elif self.demo_step == 5:
                self._demo_move_simulation()
            elif self.demo_step == 6:
                self._demo_arduino_communication()
            elif self.demo_step == 7:
                self._demo_summary()
                self.demo_timer.cancel()
                return
            
            self.demo_step += 1
            
        except Exception as e:
            self.get_logger().error(f"Demo step error: {e}")
    
    def _demo_startup_sequence(self):
        """Demo: System startup and initialization"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("📱 DEMO STEP 1: System Startup")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔧 Hardware nodes should be initializing...")
        self.get_logger().info("📺 LCD should show 'ChessMate Ready'")
        self.get_logger().info("🎛️  Rotary encoder ready for input")
        self.get_logger().info("🔌 Arduino connections established (mock mode)")
        
        self._log_demo_message("System startup completed")
    
    def _demo_menu_navigation(self):
        """Demo: Menu navigation with rotary encoder"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎛️  DEMO STEP 2: Menu Navigation")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔄 Simulating rotary encoder rotation...")
        
        # Simulate menu navigation
        directions = [1, 1, -1, 1]  # Navigate through menu options
        for i, direction in enumerate(directions):
            msg = RotaryEncoderEvent()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.event_type = RotaryEncoderEvent.EVENT_TYPE_ROTATION
            msg.direction = direction
            msg.button_pressed = False
            msg.encoder_position = i
            
            self.encoder_publisher.publish(msg)
            
            direction_text = "clockwise" if direction > 0 else "counter-clockwise"
            self.get_logger().info(f"🔄 Rotated {direction_text} - menu should update")
            
            time.sleep(0.5)
        
        self._log_demo_message("Menu navigation demonstrated")
    
    def _demo_game_setup(self):
        """Demo: Game setup menu interaction"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("⚙️  DEMO STEP 3: Game Setup Menu")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 Navigating to 'New Game' option...")
        
        # Navigate to "New Game" (assuming it's first option)
        for _ in range(2):
            msg = RotaryEncoderEvent()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.event_type = RotaryEncoderEvent.EVENT_TYPE_ROTATION
            msg.direction = -1  # Go back to first option
            msg.button_pressed = False
            msg.encoder_position = 0
            
            self.encoder_publisher.publish(msg)
            time.sleep(0.3)
        
        self.get_logger().info("📍 Should be on 'New Game' option")
        self._log_demo_message("Positioned on New Game menu option")
    
    def _demo_new_game_selection(self):
        """Demo: Selecting new game"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎮 DEMO STEP 4: New Game Selection")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔘 Pressing button to select 'New Game'...")
        
        # Simulate button press to select "New Game"
        msg = RotaryEncoderEvent()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.event_type = RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS
        msg.direction = 0
        msg.button_pressed = True
        msg.encoder_position = 0
        
        self.encoder_publisher.publish(msg)
        
        self.get_logger().info("✅ New Game selected!")
        self.get_logger().info("🎯 Game management should initialize new game")
        self.get_logger().info("📺 LCD should show game setup status")
        
        self._log_demo_message("New Game selected - game initialization triggered")
    
    def _demo_game_start(self):
        """Demo: Game start sequence"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🏁 DEMO STEP 5: Game Start Sequence")
        self.get_logger().info("=" * 60)
        self.get_logger().info("♟️  Initializing chess game...")
        self.get_logger().info("🤖 Setting up board communication...")
        self.get_logger().info("🎯 Determining player colors...")
        self.get_logger().info("⏰ Setting time controls...")
        
        # The game management node should handle the actual game setup
        # We just simulate the expected flow
        
        self.get_logger().info("✅ Game setup complete!")
        self.get_logger().info("📺 LCD should show 'Your turn (White)' or similar")
        
        self._log_demo_message("Game started - waiting for first move")
    
    def _demo_move_simulation(self):
        """Demo: Move simulation and board interaction"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("♟️  DEMO STEP 6: Move Simulation")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 Simulating chess move sequence...")
        
        # This would normally come from board sensors, but we simulate it
        self.get_logger().info("📡 Board sensors would detect piece movement")
        self.get_logger().info("🔍 Move validation through chess engine")
        self.get_logger().info("🤖 Computer calculating response...")
        self.get_logger().info("🦾 Robot arm would execute computer move")
        
        self._log_demo_message("Move sequence simulated")
    
    def _demo_arduino_communication(self):
        """Demo: Arduino communication"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔌 DEMO STEP 7: Arduino Communication")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 Arduino communication should be active...")
        self.get_logger().info("🎛️  Board controller: Monitoring piece positions")
        self.get_logger().info("🦾 Arm controller: Ready for move execution")
        self.get_logger().info("💓 Heartbeat messages maintaining connection")
        
        # Check if we received any Arduino responses
        arduino_responses = len([msg for msg in self.message_log if "Arduino" in msg])
        self.get_logger().info(f"📊 Arduino responses received: {arduino_responses}")
        
        self._log_demo_message("Arduino communication demonstrated")
    
    def _demo_summary(self):
        """Demo: Summary and conclusion"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("📋 DEMO SUMMARY: Hardware Integration Complete")
        self.get_logger().info("=" * 60)
        
        self.get_logger().info("✅ Components Demonstrated:")
        self.get_logger().info("   🎛️  Rotary Encoder: User input simulation")
        self.get_logger().info("   📺 LCD Display: Menu and status updates")
        self.get_logger().info("   🔌 Arduino Communication: Board and arm control")
        self.get_logger().info("   🎮 Game Management: Coordination and flow")
        
        self.get_logger().info("")
        self.get_logger().info("📊 Message Activity Summary:")
        for msg in self.message_log[-10:]:  # Show last 10 messages
            self.get_logger().info(f"   📝 {msg}")
        
        self.get_logger().info("")
        self.get_logger().info("🎯 Next Steps:")
        self.get_logger().info("   1. Connect real hardware (Raspberry Pi + Arduino)")
        self.get_logger().info("   2. Test with actual chess engine integration")
        self.get_logger().info("   3. Integrate with robot arm kinematics")
        self.get_logger().info("   4. Add comprehensive error handling")
        
        self.get_logger().info("")
        self.get_logger().info("🎉 Hardware Integration Demo Complete!")
        self.get_logger().info("=" * 60)
        
        # Shutdown after summary
        self.create_timer(3.0, self._shutdown_demo, one_shot=True)
    
    def _shutdown_demo(self):
        """Shutdown the demo"""
        self.get_logger().info("👋 Demo completed. Thank you!")
        self.demo_running = False
    
    def _log_demo_message(self, message: str):
        """Log a demo message with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.message_log.append(log_entry)
        self.get_logger().debug(f"📝 {log_entry}")
    
    # Callback methods to monitor system activity
    def _lcd_command_callback(self, msg: LCDCommand):
        """Monitor LCD commands"""
        cmd_types = {
            LCDCommand.CMD_CLEAR: "CLEAR",
            LCDCommand.CMD_DISPLAY_TEXT: "TEXT",
            LCDCommand.CMD_DISPLAY_MENU: "MENU",
            LCDCommand.CMD_DISPLAY_GAME_STATUS: "STATUS",
            LCDCommand.CMD_SET_BRIGHTNESS: "BRIGHTNESS"
        }
        cmd_name = cmd_types.get(msg.command_type, f"UNKNOWN({msg.command_type})")
        
        self.get_logger().info(f"📺 LCD Command: {cmd_name} - {msg.text[:30]}...")
        self._log_demo_message(f"LCD: {cmd_name} command")
    
    def _arduino_command_callback(self, msg: ArduinoCommand):
        """Monitor Arduino commands"""
        cmd_types = {
            ArduinoCommand.CMD_OCCUPANCY: "OCCUPANCY",
            ArduinoCommand.CMD_LEGAL_MOVES: "LEGAL_MOVES",
            ArduinoCommand.CMD_COMPUTER_MOVE: "COMPUTER_MOVE",
            ArduinoCommand.CMD_RESET: "RESET"
        }
        cmd_name = cmd_types.get(msg.command_type, f"UNKNOWN({msg.command_type})")
        target = "Board" if msg.target_arduino == 0 else "Arm"
        
        self.get_logger().info(f"🔌 Arduino Command to {target}: {cmd_name} - {msg.data}")
        self._log_demo_message(f"Arduino: {cmd_name} to {target}")
    
    def _arduino_response_callback(self, msg: String):
        """Monitor Arduino responses"""
        self.get_logger().info(f"📡 Arduino Response: {msg.data}")
        self._log_demo_message(f"Arduino Response: {msg.data[:20]}...")
    
    def _board_state_callback(self, msg: BoardState):
        """Monitor board state updates"""
        self.get_logger().info(f"♟️  Board State Update: {msg.fen[:20]}...")
        self._log_demo_message("Board state updated")


def main(args=None):
    """Main entry point for the hardware integration demo"""
    rclpy.init(args=args)
    
    try:
        demo = HardwareIntegrationDemo()
        
        print("\n" + "="*80)
        print("🎮 CHESSMATE HARDWARE INTEGRATION DEMO")
        print("="*80)
        print("This demo shows the complete hardware integration flow:")
        print("• User interface (rotary encoder + LCD)")
        print("• Arduino communication (board sensing + arm control)")
        print("• Game management coordination")
        print("• Message flow between all components")
        print("\nMake sure to start the hardware nodes first:")
        print("ros2 launch chessmate_hardware hardware_nodes.launch.py")
        print("="*80 + "\n")
        
        try:
            rclpy.spin(demo)
        except KeyboardInterrupt:
            demo.get_logger().info("Demo interrupted by user")
        finally:
            demo.destroy_node()
            
    except Exception as e:
        print(f"Failed to start hardware integration demo: {e}")
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
