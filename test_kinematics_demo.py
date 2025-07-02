#!/usr/bin/env python3
"""
ChessMate Kinematics Demo
Test the SCARA kinematics and chess coordinate mapping without ROS
"""

import sys
import os
import math

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/chessmate_kinematics'))

from chessmate_kinematics import SCARAKinematics, ChessBoardMapper


def test_scara_kinematics():
    """Test SCARA kinematics functionality"""
    print("=== SCARA Kinematics Test ===")
    
    scara = SCARAKinematics()
    print(f"SCARA Configuration:")
    print(f"  L1 = {scara.config.l1:.3f}m")
    print(f"  L2 = {scara.config.l2:.3f}m")
    print(f"  Workspace: {scara.workspace_radius_min:.3f}m to {scara.workspace_radius_max:.3f}m")
    print()
    
    # Test forward kinematics
    print("Forward Kinematics Tests:")
    test_configs = [
        (0.0, 0.0, 0.05),
        (math.pi/4, 0.0, 0.03),
        (0.0, math.pi/4, 0.08),
        (math.pi/6, -math.pi/6, 0.02)
    ]
    
    for i, (theta1, theta2, z) in enumerate(test_configs):
        try:
            x, y, z_out = scara.forward_kinematics(theta1, theta2, z)
            print(f"  Test {i+1}: [{theta1:.3f}, {theta2:.3f}, {z:.3f}] -> ({x:.3f}, {y:.3f}, {z_out:.3f})")
        except Exception as e:
            print(f"  Test {i+1}: ERROR - {e}")
    
    print()
    
    # Test inverse kinematics
    print("Inverse Kinematics Tests:")
    test_positions = [
        (0.300, 0.000, 0.05),  # Straight ahead
        (0.200, 0.200, 0.03),  # Diagonal
        (0.100, -0.100, 0.08), # Other diagonal
        (0.350, 0.050, 0.02)   # Near max reach
    ]
    
    for i, (x, y, z) in enumerate(test_positions):
        try:
            theta1, theta2, z_out = scara.inverse_kinematics(x, y, z)
            print(f"  Test {i+1}: ({x:.3f}, {y:.3f}, {z:.3f}) -> [{theta1:.3f}, {theta2:.3f}, {z_out:.3f}]")
            
            # Verify with forward kinematics
            x_check, y_check, z_check = scara.forward_kinematics(theta1, theta2, z_out)
            error = ((x-x_check)**2 + (y-y_check)**2 + (z-z_check)**2)**0.5
            print(f"           Verification error: {error:.6f}m")
            
        except Exception as e:
            print(f"  Test {i+1}: ERROR - {e}")
    
    print()


def test_chess_board_mapper():
    """Test chess board coordinate mapping"""
    print("=== Chess Board Mapper Test ===")
    
    mapper = ChessBoardMapper()
    print(f"Chess Board Configuration:")
    print(f"  Center: ({mapper.config.board_center_x:.3f}, {mapper.config.board_center_y:.3f})m")
    print(f"  Square size: {mapper.config.square_size:.3f}m")
    print(f"  Board size: {mapper.config.board_size:.3f}m")
    print()
    
    # Test corner squares
    print("Corner Square Mapping:")
    corners = ["a1", "a8", "h1", "h8"]
    for square in corners:
        x, y, z = mapper.square_to_cartesian(square)
        print(f"  {square}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    print()
    
    # Test center squares
    print("Center Square Mapping:")
    centers = ["d4", "d5", "e4", "e5"]
    for square in centers:
        x, y, z = mapper.square_to_cartesian(square)
        print(f"  {square}: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    print()
    
    # Test piece handling positions
    print("Piece Handling Positions for e4:")
    square = "e4"
    pickup_pos = mapper.get_pickup_position(square)
    transport_pos = mapper.get_transport_position(square)
    place_pos = mapper.get_place_position(square)
    
    print(f"  Pickup:    ({pickup_pos[0]:.3f}, {pickup_pos[1]:.3f}, {pickup_pos[2]:.3f})")
    print(f"  Transport: ({transport_pos[0]:.3f}, {transport_pos[1]:.3f}, {transport_pos[2]:.3f})")
    print(f"  Place:     ({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
    
    print()
    
    # Test move distances
    print("Move Distance Calculations:")
    moves = [("e2", "e4"), ("a1", "h8"), ("d1", "d8"), ("b1", "c3")]
    for from_sq, to_sq in moves:
        distance = mapper.calculate_move_distance(from_sq, to_sq)
        print(f"  {from_sq} -> {to_sq}: {distance:.3f}m")
    
    print()


def test_integrated_chess_kinematics():
    """Test integrated chess move to robot kinematics"""
    print("=== Integrated Chess-Robot Kinematics ===")
    
    scara = SCARAKinematics()
    mapper = ChessBoardMapper()
    
    # Test chess moves with robot kinematics
    test_moves = ["e2", "e4", "d1", "h8", "a1", "h1"]
    
    print("Chess Square -> Robot Kinematics:")
    for square in test_moves:
        try:
            # Get chess square position
            x, y, z = mapper.get_pickup_position(square)
            
            # Check if reachable
            if scara.is_reachable(x, y, z):
                # Calculate joint angles
                theta1, theta2, z_joint = scara.inverse_kinematics(x, y, z)
                print(f"  {square}: ({x:.3f}, {y:.3f}, {z:.3f}) -> [{theta1:.3f}, {theta2:.3f}, {z_joint:.3f}] ✓")
            else:
                print(f"  {square}: ({x:.3f}, {y:.3f}, {z:.3f}) -> UNREACHABLE ✗")
                
        except Exception as e:
            print(f"  {square}: ERROR - {e}")
    
    print()
    
    # Check workspace coverage of chess board
    print("Chess Board Workspace Analysis:")
    reachable_squares = 0
    total_squares = 64
    
    for rank in range(1, 9):
        for file_char in 'abcdefgh':
            square = f"{file_char}{rank}"
            x, y, z = mapper.get_pickup_position(square)
            if scara.is_reachable(x, y, z):
                reachable_squares += 1
    
    coverage = (reachable_squares / total_squares) * 100
    print(f"  Reachable squares: {reachable_squares}/{total_squares} ({coverage:.1f}%)")
    
    if coverage < 100:
        print("  ⚠️  Some chess squares are outside robot workspace!")
        print("     Consider adjusting board position or robot configuration.")
    else:
        print("  ✅ All chess squares are within robot workspace!")


def main():
    """Main demo function"""
    print("ChessMate Kinematics Demo")
    print("=" * 50)
    print()
    
    try:
        test_scara_kinematics()
        test_chess_board_mapper()
        test_integrated_chess_kinematics()
        
        print("=" * 50)
        print("✅ All kinematics tests completed successfully!")
        print("🎯 ChessMate kinematics system is ready for integration!")
        
    except Exception as e:
        print(f"❌ Demo failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
