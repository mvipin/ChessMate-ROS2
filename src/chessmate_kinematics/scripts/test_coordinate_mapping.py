#!/usr/bin/env python3
"""
Test script for chess coordinate mapping with updated F360 dimensions.

This script verifies that the coordinate mapping system correctly converts
between chess notation and robot coordinates using the real F360 measurements.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from chessmate_kinematics.chess_coordinate_mapper import ChessBoardMapper, ChessBoardConfiguration

def test_coordinate_mapping():
    """Test coordinate mapping with updated F360 dimensions"""
    
    print("=== ChessMate Coordinate Mapping Test ===")
    print("Testing with real F360 dimensions:")
    print("- Board size: 233.33mm x 233.33mm")
    print("- Square size: 29.166mm x 29.166mm") 
    print("- Board center: 253.67mm from robot base")
    print()
    
    # Create mapper with updated configuration
    mapper = ChessBoardMapper()
    config = mapper.config
    
    print("Configuration:")
    print(f"  Board center X: {config.board_center_x:.5f}m ({config.board_center_x*1000:.2f}mm)")
    print(f"  Board center Y: {config.board_center_y:.5f}m ({config.board_center_y*1000:.2f}mm)")
    print(f"  Board size: {config.board_size:.5f}m ({config.board_size*1000:.2f}mm)")
    print(f"  Square size: {config.square_size:.6f}m ({config.square_size*1000:.3f}mm)")
    print()
    
    # Test key squares
    test_squares = ['a1', 'a8', 'h1', 'h8', 'e4', 'd4', 'e5', 'd5']
    
    print("Square Coordinate Mapping:")
    print("Square | X (mm)    | Y (mm)    | Z (mm)")
    print("-------|-----------|-----------|----------")
    
    for square in test_squares:
        x, y, z = mapper.square_to_cartesian(square)
        print(f"{square:6s} | {x*1000:8.2f}  | {y*1000:8.2f}  | {z*1000:7.2f}")
    
    print()
    
    # Test reverse mapping
    print("Reverse Coordinate Mapping Test:")
    print("Testing if coordinates map back to original squares...")
    
    all_correct = True
    for square in test_squares:
        x, y, z = mapper.square_to_cartesian(square)
        reverse_square = mapper.cartesian_to_square(x, y)
        
        if reverse_square == square:
            print(f"✓ {square} -> ({x*1000:.2f}, {y*1000:.2f}) -> {reverse_square}")
        else:
            print(f"✗ {square} -> ({x*1000:.2f}, {y*1000:.2f}) -> {reverse_square}")
            all_correct = False
    
    print()
    
    # Test board boundaries
    print("Board Boundary Analysis:")
    
    # Calculate expected board boundaries
    half_board = config.board_size / 2
    min_x = config.board_center_x - half_board
    max_x = config.board_center_x + half_board
    min_y = config.board_center_y - half_board
    max_y = config.board_center_y + half_board
    
    print(f"Expected board boundaries:")
    print(f"  X: {min_x*1000:.2f}mm to {max_x*1000:.2f}mm")
    print(f"  Y: {min_y*1000:.2f}mm to {max_y*1000:.2f}mm")
    
    # Test corner squares
    a1_x, a1_y, _ = mapper.square_to_cartesian('a1')
    h8_x, h8_y, _ = mapper.square_to_cartesian('h8')
    
    print(f"Actual corner positions:")
    print(f"  A1: ({a1_x*1000:.2f}, {a1_y*1000:.2f})mm")
    print(f"  H8: ({h8_x*1000:.2f}, {h8_y*1000:.2f})mm")
    
    print()
    
    # Test center squares
    print("Center Square Analysis:")
    center_squares = ['d4', 'd5', 'e4', 'e5']
    
    for square in center_squares:
        x, y, z = mapper.square_to_cartesian(square)
        distance_from_center = ((x - config.board_center_x)**2 + (y - config.board_center_y)**2)**0.5
        print(f"  {square}: ({x*1000:.2f}, {y*1000:.2f})mm, distance from center: {distance_from_center*1000:.2f}mm")
    
    print()
    
    # Validation summary
    print("=== Validation Summary ===")
    if all_correct:
        print("✓ All coordinate mappings are consistent")
    else:
        print("✗ Some coordinate mappings failed")
    
    # Check if coordinates match URDF reference points
    print("\nURDF Reference Point Comparison:")
    print("(These should match the reference markers in the URDF)")
    
    # A1 reference (bottom-left from robot perspective)
    a1_x, a1_y, _ = mapper.square_to_cartesian('a1')
    print(f"A1 center: ({a1_x*1000:.2f}, {a1_y*1000:.2f})mm")
    
    # H8 reference (top-right from robot perspective)  
    h8_x, h8_y, _ = mapper.square_to_cartesian('h8')
    print(f"H8 center: ({h8_x*1000:.2f}, {h8_y*1000:.2f})mm")
    
    # E4 reference (center-ish)
    e4_x, e4_y, _ = mapper.square_to_cartesian('e4')
    print(f"E4 center: ({e4_x*1000:.2f}, {e4_y*1000:.2f})mm")
    
    print("\nTest completed!")

if __name__ == "__main__":
    test_coordinate_mapping()
