#!/usr/bin/env python3
"""
Test Stockfish Integration
Quick test to verify chess engine integration is working
"""

import rclpy
from rclpy.node import Node
import chess
import time

# Import our chess engine services
from chessmate_msgs.srv import GetBestMove, EvaluatePosition, ValidateMove
from chessmate_msgs.msg import ChessMove, BoardState


class StockfishTester(Node):
    """Test node for Stockfish integration"""
    
    def __init__(self):
        super().__init__('stockfish_tester')
        
        # Create service clients
        self.get_best_move_client = self.create_client(GetBestMove, 'get_best_move')
        self.evaluate_position_client = self.create_client(EvaluatePosition, 'evaluate_position')
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move')
        
        self.get_logger().info("Stockfish Tester initialized")
    
    def test_get_best_move(self):
        """Test GetBestMove service"""
        self.get_logger().info("Testing GetBestMove service...")
        
        if not self.get_best_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GetBestMove service not available")
            return False
        
        # Test with starting position
        request = GetBestMove.Request()
        request.fen_string = chess.STARTING_FEN
        request.time_limit = 1.0
        request.difficulty_level = "intermediate"
        
        try:
            future = self.get_best_move_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                response = future.result()
                self.get_logger().info(f"Best move: {response.best_move.from_square}->{response.best_move.to_square}")
                self.get_logger().info(f"Evaluation: {response.evaluation:.1f} centipawns")
                self.get_logger().info(f"Analysis time: {response.analysis_time:.3f}s")
                self.get_logger().info(f"Engine: {response.engine_info}")
                return True
            else:
                self.get_logger().error("GetBestMove service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"GetBestMove test error: {e}")
            return False
    
    def test_evaluate_position(self):
        """Test EvaluatePosition service"""
        self.get_logger().info("Testing EvaluatePosition service...")
        
        if not self.evaluate_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("EvaluatePosition service not available")
            return False
        
        # Test with a tactical position (Scholar's mate setup)
        tactical_fen = "rnbqkb1r/pppp1ppp/5n2/4p3/2B1P3/8/PPPP1PPP/RNBQK1NR w KQkq - 2 3"
        
        request = EvaluatePosition.Request()
        request.fen_string = tactical_fen
        request.analysis_time = 0.5
        request.include_best_line = True
        
        try:
            future = self.evaluate_position_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                response = future.result()
                self.get_logger().info(f"Position evaluation: {response.evaluation_text}")
                self.get_logger().info(f"Numerical eval: {response.evaluation:.1f}")
                if response.best_line:
                    self.get_logger().info(f"Best line: {response.best_line}")
                return True
            else:
                self.get_logger().error("EvaluatePosition service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"EvaluatePosition test error: {e}")
            return False
    
    def test_validate_move(self):
        """Test ValidateMove service"""
        self.get_logger().info("Testing ValidateMove service...")
        
        if not self.validate_move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ValidateMove service not available")
            return False
        
        # Test legal move
        request = ValidateMove.Request()
        request.fen_string = chess.STARTING_FEN
        
        # Create a legal move (e2-e4)
        legal_move = ChessMove()
        legal_move.from_square = "e2"
        legal_move.to_square = "e4"
        legal_move.piece_type = "pawn"
        legal_move.move_type = "normal"
        legal_move.is_capture = False
        
        request.proposed_move = legal_move
        
        try:
            future = self.validate_move_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                response = future.result()
                self.get_logger().info(f"Move validation: {response.validation_message}")
                self.get_logger().info(f"Is legal: {response.is_legal}")
                if response.resulting_fen:
                    self.get_logger().info(f"Resulting position: {response.resulting_fen[:20]}...")
                return response.is_legal
            else:
                self.get_logger().error("ValidateMove service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"ValidateMove test error: {e}")
            return False
    
    def run_all_tests(self):
        """Run all tests"""
        self.get_logger().info("=== Starting Stockfish Integration Tests ===")
        
        tests = [
            ("GetBestMove", self.test_get_best_move),
            ("EvaluatePosition", self.test_evaluate_position),
            ("ValidateMove", self.test_validate_move)
        ]
        
        results = {}
        for test_name, test_func in tests:
            self.get_logger().info(f"\n--- Running {test_name} Test ---")
            try:
                results[test_name] = test_func()
            except Exception as e:
                self.get_logger().error(f"{test_name} test failed with exception: {e}")
                results[test_name] = False
            
            time.sleep(1)  # Brief pause between tests
        
        # Print summary
        self.get_logger().info("\n=== Test Results Summary ===")
        all_passed = True
        for test_name, passed in results.items():
            status = "PASS" if passed else "FAIL"
            self.get_logger().info(f"{test_name}: {status}")
            if not passed:
                all_passed = False
        
        if all_passed:
            self.get_logger().info("🎉 All tests PASSED! Stockfish integration is working!")
        else:
            self.get_logger().error("❌ Some tests FAILED. Check the logs above.")
        
        return all_passed


def main():
    """Main test function"""
    rclpy.init()
    
    tester = StockfishTester()
    
    try:
        # Run tests
        success = tester.run_all_tests()
        
        if success:
            print("\n✅ Stockfish integration test completed successfully!")
        else:
            print("\n❌ Stockfish integration test failed!")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
