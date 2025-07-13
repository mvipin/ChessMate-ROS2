#!/usr/bin/env python3
"""
Topic-Based Game Management Node

This orchestrates the complete chess game flow using topic-based communication
instead of ROS2 services to avoid communication issues.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import chess
from chessmate_msgs.srv import CalculateMove, ExecuteMove, SetBoardMode
from chessmate_msgs.msg import GameState, ChessMove
from chessmate_hardware.topic_engine_client import TopicEngineClient
from chessmate_hardware.topic_arduino_clients import TopicRobotClient, TopicBoardClient

class TopicGameManagement(Node):
    def __init__(self):
        super().__init__('topic_game_management')
        
        # Parameters
        self.hardware_mode = self.declare_parameter('hardware_mode', 'mock').value
        self.auto_start = self.declare_parameter('auto_start', True).value
        self.skill_level = self.declare_parameter('skill_level', 5).value
        self.time_limit = self.declare_parameter('time_limit', 3.0).value
        
        # Game state
        self.chess_board = chess.Board()
        self.game_active = False
        self.current_player = 'human'  # 'human' or 'computer'
        self.move_count = 0
        self.game_history = []
        
        # Topic-based service clients (using working patterns)
        self.engine_client = TopicEngineClient(self, 'engine/calculate_move')
        self.robot_client = TopicRobotClient(self, 'robot/execute_move')
        self.board_client = TopicBoardClient(self, 'chessboard/set_mode')
        
        # Game state publisher
        self.game_state_publisher = self.create_publisher(GameState, 'game/state', 10)
        
        # Game control subscribers
        self.game_control_subscriber = self.create_subscription(
            String, 'game/control', self.handle_game_control, 10)
        
        # Human move input subscriber
        self.human_move_subscriber = self.create_subscription(
            String, 'game/human_move', self.handle_human_move, 10)
        
        self.get_logger().info("🎮 Topic-based Game Management initialized")
        self.get_logger().info(f"Hardware mode: {self.hardware_mode}")
        self.get_logger().info(f"Skill level: {self.skill_level}")
        
        # Initialize game system
        self.initialize_game_system()
    
    def initialize_game_system(self):
        """Initialize the complete game system"""
        self.get_logger().info("🔧 Initializing game system...")
        
        def init_thread():
            time.sleep(2.0)  # Brief delay for services to be ready
            
            try:
                # Wait for all services
                self.get_logger().info("⏳ Waiting for services...")
                
                if not self.engine_client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error("❌ Engine service not available")
                    return
                self.get_logger().info("✅ Engine service ready")
                
                if not self.robot_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("❌ Robot service not available")
                    return
                self.get_logger().info("✅ Robot service ready")
                
                if not self.board_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("❌ Board service not available")
                    return
                self.get_logger().info("✅ Board service ready")
                
                # Initialize board mode
                self.get_logger().info("🔧 Setting board to game mode...")
                board_request = SetBoardMode.Request()
                board_request.mode = SetBoardMode.Request.MODE_PLAYING
                board_request.force_mode_change = True
                board_request.additional_params = "New game initialization"
                
                board_response = self.board_client.call(board_request)
                if board_response and board_response.success:
                    self.get_logger().info("✅ Board initialized for game")
                else:
                    self.get_logger().warn("⚠️  Board initialization failed, continuing...")
                
                # Start game if auto_start enabled
                if self.auto_start:
                    self.start_new_game()
                else:
                    self.get_logger().info("🎮 Game system ready - waiting for start command")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Game system initialization failed: {e}")
        
        thread = threading.Thread(target=init_thread)
        thread.daemon = True
        thread.start()
    
    def handle_game_control(self, msg):
        """Handle game control commands"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            
            if command == 'start':
                self.start_new_game()
            elif command == 'stop':
                self.stop_game()
            elif command == 'reset':
                self.reset_game()
            elif command == 'status':
                self.publish_game_state()
            else:
                self.get_logger().warn(f"Unknown game command: {command}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing game control: {e}")
    
    def handle_human_move(self, msg):
        """Handle human move input"""
        if not self.game_active or self.current_player != 'human':
            self.get_logger().warn("⚠️  Human move received but not human's turn")
            return
        
        try:
            move_data = json.loads(msg.data)
            move_uci = move_data.get('move', '')
            
            self.get_logger().info(f"👤 Human move received: {move_uci}")
            self.process_human_move(move_uci)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing human move: {e}")
    
    def start_new_game(self):
        """Start a new chess game"""
        self.get_logger().info("🎮 Starting new chess game...")
        
        # Reset game state
        self.chess_board = chess.Board()
        self.game_active = True
        self.current_player = 'human'  # Human plays white, goes first
        self.move_count = 0
        self.game_history = []
        
        self.publish_game_state()
        
        self.get_logger().info("✅ New game started - waiting for human move")
        self.get_logger().info(f"Current position: {self.chess_board.fen()}")
    
    def stop_game(self):
        """Stop the current game"""
        self.get_logger().info("🛑 Stopping current game...")
        self.game_active = False
        self.publish_game_state()
    
    def reset_game(self):
        """Reset and start a new game"""
        self.stop_game()
        time.sleep(0.5)
        self.start_new_game()
    
    def process_human_move(self, move_uci):
        """Process a human move"""
        try:
            # Parse and validate move
            human_move = chess.Move.from_uci(move_uci)
            
            if human_move not in self.chess_board.legal_moves:
                self.get_logger().error(f"❌ Illegal human move: {move_uci}")
                return
            
            # Execute human move
            self.chess_board.push(human_move)
            self.move_count += 1
            
            # Record move
            self.game_history.append({
                'move_number': self.move_count,
                'player': 'human',
                'move': move_uci,
                'fen': self.chess_board.fen()
            })
            
            self.get_logger().info(f"✅ Human played: {move_uci}")
            
            # Check game end conditions
            if self.chess_board.is_game_over():
                self.handle_game_over()
                return
            
            # Switch to computer turn
            self.current_player = 'computer'
            self.publish_game_state()
            
            # Process computer move
            self.process_computer_move()
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing human move: {e}")
    
    def process_computer_move(self):
        """Process computer move using engine and robot"""
        self.get_logger().info("🧠 Processing computer move...")
        
        def computer_move_thread():
            try:
                # Get move from engine
                engine_request = CalculateMove.Request()
                engine_request.fen = self.chess_board.fen()
                engine_request.time_limit = self.time_limit
                engine_request.skill_level = self.skill_level
                engine_request.white_to_move = self.chess_board.turn
                
                self.get_logger().info("📤 Requesting move from engine...")
                engine_response = self.engine_client.call(engine_request)
                
                if not (engine_response and engine_response.success):
                    self.get_logger().error("❌ Engine calculation failed")
                    return
                
                move_uci = f"{engine_response.best_move.from_square}{engine_response.best_move.to_square}"
                computer_move = chess.Move.from_uci(move_uci)
                
                # Validate move
                if computer_move not in self.chess_board.legal_moves:
                    self.get_logger().error(f"❌ Invalid computer move: {move_uci}")
                    return
                
                self.get_logger().info(f"✅ Engine calculated: {move_uci} (eval: {engine_response.evaluation})")
                
                # Execute move via robot
                self.get_logger().info("🤖 Executing computer move...")
                
                robot_request = ExecuteMove.Request()
                robot_request.move.from_square = engine_response.best_move.from_square
                robot_request.move.to_square = engine_response.best_move.to_square
                robot_request.move.piece_type = engine_response.best_move.piece_type
                robot_request.move.is_capture = engine_response.best_move.is_capture
                
                robot_response = self.robot_client.call(robot_request)
                
                if robot_response and robot_response.success:
                    self.get_logger().info("✅ Robot move executed successfully")
                    
                    # Update game state
                    self.chess_board.push(computer_move)
                    self.move_count += 1
                    
                    # Record move
                    self.game_history.append({
                        'move_number': self.move_count,
                        'player': 'computer',
                        'move': move_uci,
                        'evaluation': engine_response.evaluation,
                        'fen': self.chess_board.fen()
                    })
                    
                    self.get_logger().info(f"✅ Computer played: {move_uci}")
                    
                    # Check game end conditions
                    if self.chess_board.is_game_over():
                        self.handle_game_over()
                        return
                    
                    # Switch to human turn
                    self.current_player = 'human'
                    self.publish_game_state()
                    
                    self.get_logger().info("👤 Waiting for human move...")
                    
                else:
                    self.get_logger().error("❌ Robot move execution failed")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Computer move processing failed: {e}")
        
        thread = threading.Thread(target=computer_move_thread)
        thread.daemon = True
        thread.start()
    
    def handle_game_over(self):
        """Handle game over conditions"""
        self.game_active = False
        
        result = "Unknown"
        if self.chess_board.is_checkmate():
            winner = "Human" if self.chess_board.turn == chess.BLACK else "Computer"
            result = f"Checkmate - {winner} wins!"
        elif self.chess_board.is_stalemate():
            result = "Stalemate - Draw"
        elif self.chess_board.is_insufficient_material():
            result = "Insufficient material - Draw"
        elif self.chess_board.is_fivefold_repetition():
            result = "Fivefold repetition - Draw"
        
        self.get_logger().info(f"🏁 Game Over: {result}")
        self.get_logger().info(f"Total moves: {self.move_count}")
        
        self.publish_game_state()
    
    def publish_game_state(self):
        """Publish current game state"""
        try:
            game_state = GameState()
            game_state.fen = self.chess_board.fen()
            game_state.status = GameState.STATUS_PLAYING if self.game_active else GameState.STATUS_WAITING
            game_state.white_to_move = self.chess_board.turn
            game_state.move_number = self.chess_board.fullmove_number
            game_state.halfmove_clock = self.chess_board.halfmove_clock

            # Set result if game is over
            if self.chess_board.is_game_over():
                game_state.status = GameState.STATUS_FINISHED
                if self.chess_board.is_checkmate():
                    game_state.result = GameState.RESULT_WHITE_WINS if not self.chess_board.turn else GameState.RESULT_BLACK_WINS
                else:
                    game_state.result = GameState.RESULT_DRAW
            else:
                game_state.result = GameState.RESULT_NONE

            # Set player names
            game_state.white_player = "Human"
            game_state.black_player = "ChessMate"
            game_state.event_name = "ChessMate Game"

            # Time control (not implemented yet)
            game_state.time_control_enabled = False
            game_state.white_time_remaining = 0.0
            game_state.black_time_remaining = 0.0

            self.game_state_publisher.publish(game_state)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing game state: {e}")

def main():
    rclpy.init()
    
    try:
        game_manager = TopicGameManagement()
        rclpy.spin(game_manager)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            game_manager.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
