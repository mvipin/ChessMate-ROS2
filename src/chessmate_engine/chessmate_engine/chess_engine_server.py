#!/usr/bin/env python3
"""
Chess Engine Server
Standalone ROS2 node providing chess engine services without game management
"""

import rclpy
from rclpy.node import Node
import chess

# ROS2 message imports
from chessmate_msgs.msg import BoardState, ChessMove
from chessmate_msgs.srv import (
    GetBestMove, EvaluatePosition, ValidateMove, UpdateGameState
)

# Local imports
from .stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
from .message_converters import MessageConverter


class ChessEngineServer(Node):
    """
    Chess Engine Server Node
    
    Provides chess engine services without game state management:
    - GetBestMove: Get best move for a position
    - EvaluatePosition: Evaluate a chess position
    - ValidateMove: Check if a move is legal
    - UpdateGameState: Update engine configuration
    """
    
    def __init__(self):
        super().__init__('chess_engine_server')
        
        # Initialize components
        self.stockfish = StockfishInterface()
        self.converter = MessageConverter()
        
        # ROS2 Services
        self.get_best_move_service = self.create_service(
            GetBestMove, 'get_best_move', self.get_best_move_callback)
        
        self.evaluate_position_service = self.create_service(
            EvaluatePosition, 'evaluate_position', self.evaluate_position_callback)
        
        self.validate_move_service = self.create_service(
            ValidateMove, 'validate_move', self.validate_move_callback)
        
        self.update_game_state_service = self.create_service(
            UpdateGameState, 'update_game_state', self.update_game_state_callback)
        
        # Initialize engine
        if not self.stockfish.initialize():
            self.get_logger().error("Failed to initialize Stockfish engine")
        else:
            self.get_logger().info("Chess Engine Server initialized successfully")
    
    def get_best_move_callback(self, request, response):
        """Handle GetBestMove service requests"""
        try:
            self.get_logger().info(f"GetBestMove request: {request.fen_string[:20]}...")
            
            # Configure engine based on difficulty
            if request.difficulty_level:
                try:
                    difficulty = DifficultyLevel(request.difficulty_level)
                    config = EngineConfig.from_difficulty(difficulty)
                    self.stockfish.config = config
                    self.get_logger().info(f"Set difficulty: {request.difficulty_level}")
                except ValueError:
                    self.get_logger().warn(f"Invalid difficulty level: {request.difficulty_level}")
            
            # Get analysis
            analysis = self.stockfish.get_best_move(
                request.fen_string,
                request.time_limit if request.time_limit > 0 else None,
                request.depth_limit if request.depth_limit > 0 else None
            )
            
            if analysis.best_move:
                # Create a minimal board state for conversion
                board_state = BoardState()
                board_state.fen_string = request.fen_string
                board_state = self.converter.fen_to_board_state(request.fen_string)
                
                # Convert to ChessMove
                response.best_move = self.converter.uci_to_chess_move(
                    analysis.best_move, board_state)
                response.best_move.confidence = min(1.0, abs(analysis.evaluation) / 100.0)
                
                self.get_logger().info(f"Best move: {analysis.best_move} (eval: {analysis.evaluation:.1f})")
            else:
                self.get_logger().warn("No best move found")
            
            response.evaluation = analysis.evaluation
            response.analysis_time = analysis.analysis_time
            response.nodes_searched = analysis.nodes_searched
            response.engine_info = self.stockfish.get_engine_info()
            
        except Exception as e:
            self.get_logger().error(f"GetBestMove service error: {e}")
            response.engine_info = f"Error: {e}"
        
        return response
    
    def evaluate_position_callback(self, request, response):
        """Handle EvaluatePosition service requests"""
        try:
            self.get_logger().info(f"EvaluatePosition request: {request.fen_string[:20]}...")
            
            analysis = self.stockfish.evaluate_position(
                request.fen_string, request.analysis_time)
            
            response.evaluation = analysis.evaluation
            response.mate_in = analysis.mate_in or 0.0
            
            # Generate evaluation text
            if analysis.mate_in:
                if analysis.mate_in > 0:
                    response.evaluation_text = f"White mates in {analysis.mate_in}"
                else:
                    response.evaluation_text = f"Black mates in {abs(analysis.mate_in)}"
            elif abs(analysis.evaluation) < 50:
                response.evaluation_text = "Position is equal"
            elif analysis.evaluation > 0:
                response.evaluation_text = f"White is better (+{analysis.evaluation/100:.1f})"
            else:
                response.evaluation_text = f"Black is better ({analysis.evaluation/100:.1f})"
            
            # Include best line if requested
            if request.include_best_line and analysis.principal_variation:
                response.best_line = " ".join(analysis.principal_variation)
            
            # Check for game ending conditions
            game_over, result = self.stockfish.is_game_over(request.fen_string)
            response.is_checkmate = game_over and result in ["1-0", "0-1"]
            response.is_stalemate = game_over and result == "1/2-1/2"
            
            response.engine_info = self.stockfish.get_engine_info()
            
            self.get_logger().info(f"Position evaluation: {response.evaluation_text}")
            
        except Exception as e:
            self.get_logger().error(f"EvaluatePosition service error: {e}")
            response.evaluation_text = f"Error: {e}"
        
        return response
    
    def validate_move_callback(self, request, response):
        """Handle ValidateMove service requests"""
        try:
            self.get_logger().info(f"ValidateMove request: {request.proposed_move.from_square}->{request.proposed_move.to_square}")
            
            # Convert ChessMove to UCI
            uci_move = self.converter.chess_move_to_uci(request.proposed_move)
            
            # Validate move
            is_legal, message, resulting_fen = self.stockfish.validate_move(
                request.fen_string, uci_move)
            
            response.is_legal = is_legal
            response.validation_message = message
            
            if resulting_fen:
                response.resulting_fen = resulting_fen
                
                # Check for check/checkmate
                temp_stockfish = StockfishInterface()
                if temp_stockfish.initialize():
                    temp_stockfish.set_position(resulting_fen)
                    
                    response.is_check = temp_stockfish.board.is_check()
                    response.is_checkmate = temp_stockfish.board.is_checkmate()
                    
                    temp_stockfish.shutdown()
                
                # Check if move is capture
                response.is_capture = request.proposed_move.is_capture
            
            self.get_logger().info(f"Move validation: {message}")
            
        except Exception as e:
            self.get_logger().error(f"ValidateMove service error: {e}")
            response.is_legal = False
            response.validation_message = f"Error: {e}"
        
        return response
    
    def update_game_state_callback(self, request, response):
        """Handle UpdateGameState service requests"""
        try:
            self.get_logger().info(f"UpdateGameState request: mode={request.game_mode}")
            
            if request.reset_game:
                # Reset engine to starting position
                self.stockfish.set_position(chess.STARTING_FEN)
                response.current_fen = chess.STARTING_FEN
                response.move_number = 1
                response.active_player = "white"
                response.game_result = "*"
            else:
                # Update with provided state
                response.current_fen = request.board_state.fen_string
                response.move_number = request.board_state.fullmove_number
                response.active_player = request.board_state.active_color
                
                # Check if game is over
                game_over, result = self.stockfish.is_game_over(
                    request.board_state.fen_string)
                response.game_over = game_over
                response.game_result = result
            
            response.success = True
            response.status_message = "Game state updated successfully"
            
            self.get_logger().info(f"Game state updated: {response.status_message}")
            
        except Exception as e:
            self.get_logger().error(f"UpdateGameState service error: {e}")
            response.success = False
            response.status_message = f"Error: {e}"
        
        return response
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stockfish.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    engine_server = ChessEngineServer()
    
    try:
        rclpy.spin(engine_server)
    except KeyboardInterrupt:
        pass
    finally:
        engine_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
