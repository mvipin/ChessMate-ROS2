#!/usr/bin/env python3
"""
Test script to verify BoardState creation works properly
"""

import sys
import os

# Add the correct install path
install_path = os.path.join(os.path.dirname(__file__), 'install_arm/lib/python3.11/site-packages')
sys.path.insert(0, install_path)

# Also source the ROS setup
os.environ['AMENT_PREFIX_PATH'] = os.path.join(os.path.dirname(__file__), 'install_arm')
os.environ['PYTHONPATH'] = install_path + ':' + os.environ.get('PYTHONPATH', '')

try:
    from chessmate_msgs.msg import BoardState, ChessPiece
    print("✅ Successfully imported BoardState and ChessPiece")
    
    # Test creating a BoardState
    board_state = BoardState()
    print("✅ Successfully created BoardState object")
    
    # Test setting timestamp
    board_state.timestamp = 123456789
    print("✅ Successfully set timestamp")
    
    # Test creating squares array - must create all 64 pieces first
    squares = []
    for i in range(64):
        piece = ChessPiece()
        piece.type = "none"  # Use "none" for empty squares
        piece.color = "empty"
        piece.has_moved = False
        piece.square_id = i
        squares.append(piece)

    # Now set the complete array
    board_state.squares = squares
    
    print(f"✅ Successfully created squares array with {len(board_state.squares)} pieces")
    
    # Test setting other fields
    board_state.active_color = "white"
    board_state.castling_rights = "KQkq"
    board_state.en_passant_target = "-"
    board_state.halfmove_clock = 0
    board_state.fullmove_number = 1
    board_state.fen_string = ""
    
    print("✅ Successfully set all BoardState fields")
    print("✅ BoardState creation test PASSED")
    
except Exception as e:
    print(f"❌ BoardState creation test FAILED: {e}")
    import traceback
    traceback.print_exc()
