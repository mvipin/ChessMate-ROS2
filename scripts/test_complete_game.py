#!/usr/bin/env python3
"""
Complete ChessMate Game Simulation Test

This script simulates a complete chess game using both controllers:
- ChessBoard Controller: Handles board sensing and human moves
- Robot Controller: Handles computer moves and piece manipulation

New Architecture Test:
Pi Host → ChessBoard Controller (USB) → Human move detection
Pi Host → Robot Controller (USB) → Computer move execution
"""

import serial
import threading
import time
import sys
import os
from datetime import datetime

# Configuration - Updated paths for scripts directory
CHESSBOARD_PORT = '/dev/ttyACM0'
ROBOT_PORT = '/dev/ttyACM1'
BAUD_RATE = 9600
TIMEOUT = 1.0

class GameController:
    def __init__(self):
        self.chessboard = None
        self.robot = None
        self.running = False
        self.game_state = "INIT"
        self.move_count = 0
        
    def connect_controllers(self):
        """Connect to both controllers"""
        try:
            print("🔌 Connecting to ChessBoard controller...")
            self.chessboard = serial.Serial(CHESSBOARD_PORT, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2)
            print(f"✅ ChessBoard connected at {CHESSBOARD_PORT}")
            
            print("🔌 Connecting to Robot controller...")
            self.robot = serial.Serial(ROBOT_PORT, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2)
            print(f"✅ Robot connected at {ROBOT_PORT}")
            
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def disconnect_controllers(self):
        """Disconnect from controllers"""
        if self.chessboard and self.chessboard.is_open:
            self.chessboard.close()
            print("🔌 ChessBoard disconnected")
            
        if self.robot and self.robot.is_open:
            self.robot.close()
            print("🔌 Robot disconnected")
    
    def send_to_chessboard(self, command):
        """Send command to ChessBoard controller"""
        if not self.chessboard or not self.chessboard.is_open:
            print("❌ ChessBoard not connected")
            return False
            
        try:
            print(f"📤 ChessBoard: {command}")
            self.chessboard.write((command + '\n').encode())
            self.chessboard.flush()
            return True
        except Exception as e:
            print(f"❌ ChessBoard send error: {e}")
            return False
    
    def send_to_robot(self, command):
        """Send command to Robot controller"""
        if not self.robot or not self.robot.is_open:
            print("❌ Robot not connected")
            return False
            
        try:
            print(f"📤 Robot: {command}")
            self.robot.write((command + '\n').encode())
            self.robot.flush()
            return True
        except Exception as e:
            print(f"❌ Robot send error: {e}")
            return False
    
    def read_from_chessboard(self, timeout=5.0):
        """Read response from ChessBoard controller"""
        start_time = time.time()
        responses = []
        
        while time.time() - start_time < timeout:
            if self.chessboard.in_waiting > 0:
                line = self.chessboard.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"📥 ChessBoard: {line}")
                    responses.append(line)
                    
                    # Check for move response (4-character move like "e2e4")
                    if len(line) == 4 and line.isalnum():
                        return line  # Return the move
            time.sleep(0.1)
        
        return None
    
    def read_from_robot(self, timeout=5.0):
        """Read response from Robot controller"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.robot.in_waiting > 0:
                line = self.robot.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"📥 Robot: {line}")
                    
                    # Check for completion message
                    if "Move completed" in line or "Ready for next command" in line:
                        return True
            time.sleep(0.1)
        
        return False
    
    def initialize_game(self):
        """Initialize both controllers for a new game"""
        print("\n🎯 Initializing Game")
        print("=" * 50)
        
        # Initialize ChessBoard
        print("🔧 Initializing ChessBoard...")
        self.send_to_chessboard("mode:mock")
        time.sleep(1)
        self.send_to_chessboard("init")
        time.sleep(1)
        
        # Set initial board position
        initial_occupancy = "occupancy:0:1:2:3:4:5:6:7:48:49:50:51:52:53:54:55"
        self.send_to_chessboard(initial_occupancy)
        time.sleep(1)
        
        # Initialize Robot
        print("🔧 Initializing Robot...")
        self.send_to_robot("mode:mock")
        time.sleep(1)
        self.send_to_robot("init")
        time.sleep(1)
        self.send_to_robot("home")
        time.sleep(2)
        
        print("✅ Game initialization complete")
        self.game_state = "READY"
    
    def simulate_human_turn(self, legal_moves):
        """Simulate a human turn"""
        print(f"\n👤 Human Turn {self.move_count + 1}")
        print("=" * 30)
        
        # Send legal moves to ChessBoard
        legal_moves_cmd = "legal:" + ":".join(legal_moves)
        self.send_to_chessboard(legal_moves_cmd)
        time.sleep(1)
        
        # Start human turn
        self.send_to_chessboard("start")
        
        # Wait for human move (mock mode will simulate this)
        print("⏳ Waiting for human move...")
        human_move = self.read_from_chessboard(timeout=10.0)
        
        if human_move:
            print(f"✅ Human played: {human_move}")
            return human_move
        else:
            print("❌ No human move received")
            return None
    
    def simulate_computer_turn(self, computer_move):
        """Simulate a computer turn"""
        print(f"\n🤖 Computer Turn {self.move_count + 1}")
        print("=" * 30)
        
        # Convert move to robot format (e.g., "e2e4" -> "e2pe4p")
        robot_move = self.convert_to_robot_format(computer_move)
        
        print(f"🎯 Computer plays: {computer_move} (robot format: {robot_move})")
        
        # Send move to robot
        self.send_to_robot(robot_move)
        
        # Wait for robot to complete move
        print("⏳ Waiting for robot to execute move...")
        completed = self.read_from_robot(timeout=15.0)
        
        if completed:
            print("✅ Robot move completed")
            return True
        else:
            print("❌ Robot move failed or timed out")
            return False
    
    def convert_to_robot_format(self, move):
        """Convert chess move to robot format"""
        # Simple conversion: e2e4 -> e2pe4p (assuming pawn moves for demo)
        if len(move) == 4:
            return f"{move[0:2]}p{move[2:4]}p"
        return move
    
    def simulate_complete_game(self):
        """Simulate a complete chess game"""
        print("\n🏁 Starting Complete Game Simulation")
        print("=" * 50)
        
        # Sample opening moves
        game_moves = [
            (["e2e4", "d2d4"], "e2e4"),  # Human legal moves, Computer response
            (["e7e5", "d7d5"], "e7e5"),  # Human legal moves, Computer response
            (["g1f3", "f1c4"], "g1f3"), # Human legal moves, Computer response
            (["b8c6", "g8f6"], "b8c6"), # Human legal moves, Computer response
        ]
        
        for turn, (human_legal, computer_move) in enumerate(game_moves):
            self.move_count = turn
            
            # Human turn
            human_move = self.simulate_human_turn(human_legal)
            if not human_move:
                print("❌ Game aborted - human move failed")
                return False
            
            time.sleep(2)  # Brief pause between turns
            
            # Computer turn
            if not self.simulate_computer_turn(computer_move):
                print("❌ Game aborted - computer move failed")
                return False
            
            time.sleep(2)  # Brief pause between turns
            
            print(f"\n✅ Turn {turn + 1} completed")
        
        print("\n🎉 Complete game simulation finished!")
        return True
    
    def test_individual_controllers(self):
        """Test each controller individually"""
        print("\n🧪 Testing Individual Controllers")
        print("=" * 50)
        
        # Test ChessBoard
        print("🔧 Testing ChessBoard Controller...")
        self.send_to_chessboard("mode:mock")
        time.sleep(1)
        self.send_to_chessboard("init")
        time.sleep(1)
        print("✅ ChessBoard test complete")
        
        # Test Robot
        print("🔧 Testing Robot Controller...")
        self.send_to_robot("mode:mock")
        time.sleep(1)
        self.send_to_robot("status")
        time.sleep(1)
        print("✅ Robot test complete")

def main():
    """Main test function"""
    print("🏁 ChessMate Complete Game Test")
    print("=" * 50)
    
    # Check if ports exist
    if not os.path.exists(CHESSBOARD_PORT):
        print(f"❌ ChessBoard controller not found at {CHESSBOARD_PORT}")
        return 1
    
    if not os.path.exists(ROBOT_PORT):
        print(f"❌ Robot controller not found at {ROBOT_PORT}")
        return 1
    
    game = GameController()
    
    try:
        # Connect to controllers
        if not game.connect_controllers():
            return 1
        
        # Run tests based on command line argument
        if len(sys.argv) > 1:
            if sys.argv[1] == 'individual':
                game.test_individual_controllers()
            elif sys.argv[1] == 'init':
                game.initialize_game()
            elif sys.argv[1] == 'full':
                game.initialize_game()
                time.sleep(2)
                game.simulate_complete_game()
            else:
                print("Usage: python3 test_complete_game.py [individual|init|full]")
        else:
            # Default: run initialization and ask user
            game.initialize_game()
            
            response = input("\nRun complete game simulation? (y/n): ")
            if response.lower().startswith('y'):
                game.simulate_complete_game()
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    finally:
        game.disconnect_controllers()
    
    return 0

if __name__ == "__main__":
    exit(main())
