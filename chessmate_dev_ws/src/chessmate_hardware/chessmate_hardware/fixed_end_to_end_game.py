#!/usr/bin/env python3
"""
Fixed End-to-End Game Test

This version uses threading instead of ROS2 timers to avoid breaking
topic-based communication.
"""

import rclpy
from rclpy.node import Node
import time
import threading
import chess
import random
from chessmate_msgs.srv import CalculateMove
from chessmate_hardware.topic_engine_client import TopicEngineClient

class FixedEndToEndGameTest(Node):
    def __init__(self):
        super().__init__('fixed_end_to_end_game')
        
        # Test parameters
        self.num_moves = self.declare_parameter('num_moves', 5).value
        self.game_timeout = self.declare_parameter('game_timeout', 120).value
        
        # Initialize chess board
        self.chess_board = chess.Board()
        
        # Create topic-based engine client (works without ROS2 timers)
        self.engine_client = TopicEngineClient(self, 'engine/calculate_move')
        
        # Test state
        self.move_count = 0
        self.test_completed = False
        self.test_results = {
            'moves_played': [],
            'total_time': 0,
            'success': False,
            'errors': []
        }
        
        self.get_logger().info("🧪 Fixed End-to-End Game Test initialized")
        self.get_logger().info(f"Target: {self.num_moves} moves, Timeout: {self.game_timeout}s")
        
        # Start game using threading instead of ROS2 timer
        self.start_game_delayed()
    
    def start_game_delayed(self):
        """Start game after brief delay using threading"""
        def delayed_start():
            time.sleep(3.0)  # Brief delay for initialization
            self.run_complete_game()
        
        thread = threading.Thread(target=delayed_start)
        thread.daemon = True
        thread.start()
    
    def run_complete_game(self):
        """Run complete game without ROS2 timers"""
        self.get_logger().info("🚀 Starting complete chess game...")
        
        start_time = time.time()
        
        try:
            # Wait for engine service
            if not self.engine_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error("❌ Engine service not available")
                return
            
            self.get_logger().info("✅ Engine service ready")
            
            # Play the game
            while self.move_count < self.num_moves:
                if (time.time() - start_time) > self.game_timeout:
                    self.get_logger().error("⏰ Game timeout reached")
                    break
                
                self.get_logger().info(f"=== Move {self.move_count + 1}/{self.num_moves} ===")
                
                if self.chess_board.turn:  # White's turn (human)
                    if not self.simulate_human_move():
                        break
                else:  # Black's turn (computer)
                    if not self.calculate_computer_move():
                        break
                
                # Brief pause between moves
                time.sleep(0.5)
            
            # Complete the test
            self.test_results['total_time'] = time.time() - start_time
            self.test_results['success'] = self.move_count >= self.num_moves
            self.test_completed = True
            
            self.get_logger().info("🏁 Game completed!")
            self.get_logger().info(f"Moves completed: {self.move_count}/{self.num_moves}")
            self.get_logger().info(f"Total time: {self.test_results['total_time']:.1f}s")
            self.get_logger().info(f"Result: {'✅ SUCCESS' if self.test_results['success'] else '❌ FAILED'}")
            
            # Shutdown after brief delay
            def delayed_shutdown():
                time.sleep(2.0)
                rclpy.shutdown()
            
            shutdown_thread = threading.Thread(target=delayed_shutdown)
            shutdown_thread.daemon = True
            shutdown_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"❌ Game failed: {e}")
            self.test_results['errors'].append(str(e))
            self.test_results['success'] = False
    
    def simulate_human_move(self):
        """Simulate human move"""
        self.get_logger().info("👤 Simulating human move...")
        
        # Simulate thinking time
        thinking_time = random.uniform(1.0, 3.0)
        self.get_logger().info(f"Human thinking for {thinking_time:.1f}s...")
        time.sleep(thinking_time)
        
        # Pick a random legal move
        legal_moves = list(self.chess_board.legal_moves)
        if not legal_moves:
            self.get_logger().error("No legal moves for human")
            return False
        
        human_move = random.choice(legal_moves)
        self.chess_board.push(human_move)
        
        move_uci = f"{chess.square_name(human_move.from_square)}{chess.square_name(human_move.to_square)}"
        self.get_logger().info(f"Human played: {move_uci}")
        
        # Record move
        self.test_results['moves_played'].append({
            'move_number': self.move_count + 1,
            'player': 'human',
            'move': move_uci,
            'thinking_time': thinking_time
        })
        
        self.move_count += 1
        return True
    
    def calculate_computer_move(self):
        """Calculate computer move using topic-based engine"""
        self.get_logger().info("🧠 Calculating computer move using topic-based engine...")
        
        # Create request
        request = CalculateMove.Request()
        request.fen = self.chess_board.fen()
        request.time_limit = 2.0
        request.skill_level = 5
        request.white_to_move = self.chess_board.turn
        
        self.get_logger().info(f"   FEN: {request.fen[:50]}...")
        self.get_logger().info(f"   Time limit: {request.time_limit}s")
        self.get_logger().info(f"   Skill level: {request.skill_level}")
        
        try:
            # Use topic-based engine (works without ROS2 timers)
            response = self.engine_client.call(request)
            
            if response and response.success:
                move_uci = f"{response.best_move.from_square}{response.best_move.to_square}"
                computer_move = chess.Move.from_uci(move_uci)
                
                # Validate move
                if computer_move in self.chess_board.legal_moves:
                    self.chess_board.push(computer_move)
                    
                    self.get_logger().info(f"✅ Topic-based engine calculated move: {move_uci}")
                    self.get_logger().info(f"   Evaluation: {response.evaluation}")
                    self.get_logger().info(f"   Calculation time: {response.calculation_time:.3f}s")
                    
                    # Record move
                    self.test_results['moves_played'].append({
                        'move_number': self.move_count + 1,
                        'player': 'computer',
                        'move': move_uci,
                        'evaluation': response.evaluation,
                        'calculation_time': response.calculation_time
                    })
                    
                    self.move_count += 1
                    return True
                else:
                    self.get_logger().error(f"❌ Invalid computer move: {move_uci}")
                    return False
            else:
                self.get_logger().error("❌ Topic-based engine returned unsuccessful response")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Topic-based engine call failed: {e}")
            return False

def main():
    rclpy.init()
    
    try:
        test_node = FixedEndToEndGameTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            test_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
