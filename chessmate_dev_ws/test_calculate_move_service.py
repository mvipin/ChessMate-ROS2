#!/usr/bin/env python3
"""
Simple test script to verify the CalculateMove service works
"""

import rclpy
from rclpy.node import Node
from chessmate_msgs.srv import CalculateMove

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        self.client = self.create_client(CalculateMove, 'engine/calculate_move')
        
    def test_service(self):
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service not available")
            return False
            
        # Create request
        request = CalculateMove.Request()
        request.fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"
        request.time_limit = 2.0
        request.skill_level = 5
        request.white_to_move = False
        
        self.get_logger().info("Sending service request...")
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        
        if future.result() is None:
            self.get_logger().error("Service call timed out")
            return False
            
        result = future.result()
        self.get_logger().info(f"Service response: success={result.success}")
        if result.success:
            self.get_logger().info(f"Best move: {result.best_move.from_square} -> {result.best_move.to_square}")
            self.get_logger().info(f"Evaluation: {result.evaluation}")
        else:
            self.get_logger().error(f"Service failed: {result.message}")
            
        return result.success

def main():
    rclpy.init()
    tester = ServiceTester()
    
    try:
        success = tester.test_service()
        if success:
            print("✅ Service test PASSED")
        else:
            print("❌ Service test FAILED")
    except Exception as e:
        print(f"❌ Service test ERROR: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
