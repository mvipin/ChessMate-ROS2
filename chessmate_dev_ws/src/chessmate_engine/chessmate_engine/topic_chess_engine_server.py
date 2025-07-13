#!/usr/bin/env python3
"""
Topic-Based Chess Engine Server

This provides the same functionality as chess_engine_server but uses
topic-based communication instead of ROS2 services to work around
service communication issues.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import chess
import chess.engine
import threading
import os

class TopicChessEngineServer(Node):
    def __init__(self):
        super().__init__('topic_chess_engine_server')
        
        # Topic-based service interface
        self.request_subscriber = self.create_subscription(
            String, 'engine/calculate_move_request', self.handle_calculate_move_request, 10)
        
        self.response_publisher = self.create_publisher(
            String, 'engine/calculate_move_response', 10)
        
        # Initialize Stockfish engine
        self.engine = None
        self.engine_lock = threading.Lock()
        self.initialize_engine()
        
        self.get_logger().info("🧠 Topic-based Chess Engine Server initialized")
        
    def initialize_engine(self):
        """Initialize Stockfish chess engine"""
        try:
            # Try different Stockfish paths
            stockfish_paths = [
                '/usr/games/stockfish',
                '/usr/bin/stockfish', 
                '/usr/local/bin/stockfish',
                'stockfish'
            ]
            
            for path in stockfish_paths:
                try:
                    self.engine = chess.engine.SimpleEngine.popen_uci(path)
                    self.get_logger().info(f"✅ Stockfish initialized at: {path}")
                    return
                except:
                    continue
                    
            raise Exception("Stockfish not found in any standard location")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Stockfish: {e}")
            self.engine = None
    
    def handle_calculate_move_request(self, msg):
        """Handle calculate move request via topic"""
        try:
            # Parse request
            request_data = json.loads(msg.data)
            request_id = request_data.get('id', 'unknown')
            
            self.get_logger().info(f"📨 Processing calculate move request {request_id}")
            
            # Extract request parameters
            fen = request_data.get('fen', '')
            time_limit = request_data.get('time_limit', 2.0)
            skill_level = request_data.get('skill_level', 5)
            
            # Calculate move
            response_data = self.calculate_move(fen, time_limit, skill_level)
            response_data['id'] = request_id
            
            # Publish response
            response_msg = String()
            response_msg.data = json.dumps(response_data)
            self.response_publisher.publish(response_msg)
            
            self.get_logger().info(f"✅ Sent response for request {request_id}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing request: {e}")
            
            # Send error response
            error_response = {
                'id': request_data.get('id', 'unknown'),
                'success': False,
                'message': f"Error: {e}",
                'best_move': {
                    'from_square': '',
                    'to_square': '',
                    'piece_type': '',
                    'promotion_piece': '',
                    'is_capture': False,
                    'move_type': 'error'
                },
                'evaluation': 0.0,
                'calculation_time': 0.0
            }
            
            error_msg = String()
            error_msg.data = json.dumps(error_response)
            self.response_publisher.publish(error_msg)
    
    def calculate_move(self, fen, time_limit, skill_level):
        """Calculate best move using Stockfish"""
        start_time = time.time()
        
        try:
            if not self.engine:
                raise Exception("Stockfish engine not available")
            
            with self.engine_lock:
                # Set up board
                board = chess.Board(fen)
                
                # Configure engine
                self.engine.configure({"Skill Level": skill_level})
                
                # Calculate move
                result = self.engine.play(
                    board, 
                    chess.engine.Limit(time=time_limit)
                )
                
                if result.move is None:
                    raise Exception("No legal moves available")
                
                # Get evaluation
                info = self.engine.analyse(board, chess.engine.Limit(time=0.1))
                evaluation = info.get("score", chess.engine.PovScore(chess.engine.Cp(0), chess.WHITE)).relative.score(mate_score=10000)
                
                calculation_time = time.time() - start_time
                
                return {
                    'success': True,
                    'message': f"Best move calculated: {result.move}",
                    'best_move': {
                        'from_square': chess.square_name(result.move.from_square),
                        'to_square': chess.square_name(result.move.to_square),
                        'piece_type': board.piece_at(result.move.from_square).symbol().lower(),
                        'promotion_piece': chess.piece_name(result.move.promotion) if result.move.promotion else '',
                        'is_capture': board.is_capture(result.move),
                        'move_type': 'normal'
                    },
                    'evaluation': float(evaluation) if evaluation else 0.0,
                    'calculation_time': calculation_time
                }
                
        except Exception as e:
            calculation_time = time.time() - start_time
            self.get_logger().error(f"❌ Engine calculation failed: {e}")
            
            return {
                'success': False,
                'message': f"Calculation failed: {e}",
                'best_move': {
                    'from_square': '',
                    'to_square': '',
                    'piece_type': '',
                    'promotion_piece': '',
                    'is_capture': False,
                    'move_type': 'error'
                },
                'evaluation': 0.0,
                'calculation_time': calculation_time
            }
    
    def destroy_node(self):
        """Clean up engine on shutdown"""
        if self.engine:
            try:
                self.engine.quit()
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    
    try:
        engine_server = TopicChessEngineServer()
        rclpy.spin(engine_server)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            engine_server.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
