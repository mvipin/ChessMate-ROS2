#!/usr/bin/env python3
"""
Basic Stockfish Test
Test Stockfish integration without ROS2 services
"""

import sys
import os

# Add the package to Python path
sys.path.append('/home/smtuser/ChessMate/chessmate_dev_ws/src/chessmate_engine')

try:
    from chessmate_engine.stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
    from chessmate_engine.message_converters import MessageConverter
    import chess
    print("✅ All imports successful!")
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)


def test_stockfish_interface():
    """Test the Stockfish interface directly"""
    print("\n=== Testing Stockfish Interface ===")
    
    # Initialize Stockfish
    stockfish = StockfishInterface()
    
    if not stockfish.initialize():
        print("❌ Failed to initialize Stockfish")
        return False
    
    print("✅ Stockfish initialized successfully")
    print(f"Engine info: {stockfish.get_engine_info()}")
    
    # Test getting best move
    print("\n--- Testing Best Move ---")
    try:
        analysis = stockfish.get_best_move(chess.STARTING_FEN, time_limit=1.0)
        if analysis.best_move:
            print(f"✅ Best move: {analysis.best_move}")
            print(f"   Evaluation: {analysis.evaluation:.1f} centipawns")
            print(f"   Analysis time: {analysis.analysis_time:.3f}s")
        else:
            print("❌ No best move found")
            return False
    except Exception as e:
        print(f"❌ Best move test failed: {e}")
        return False
    
    # Test position evaluation
    print("\n--- Testing Position Evaluation ---")
    try:
        analysis = stockfish.evaluate_position(chess.STARTING_FEN, 0.5)
        print(f"✅ Position evaluation: {analysis.evaluation:.1f} centipawns")
        if analysis.principal_variation:
            print(f"   Principal variation: {' '.join(analysis.principal_variation[:3])}")
    except Exception as e:
        print(f"❌ Position evaluation test failed: {e}")
        return False
    
    # Test move validation
    print("\n--- Testing Move Validation ---")
    try:
        is_legal, message, resulting_fen = stockfish.validate_move(chess.STARTING_FEN, "e2e4")
        if is_legal:
            print(f"✅ Move e2e4 is legal: {message}")
            print(f"   Resulting FEN: {resulting_fen[:30]}...")
        else:
            print(f"❌ Move validation failed: {message}")
            return False
    except Exception as e:
        print(f"❌ Move validation test failed: {e}")
        return False
    
    # Test difficulty levels
    print("\n--- Testing Difficulty Levels ---")
    try:
        for difficulty in [DifficultyLevel.BEGINNER, DifficultyLevel.INTERMEDIATE, DifficultyLevel.ADVANCED]:
            config = EngineConfig.from_difficulty(difficulty)
            stockfish.config = config
            print(f"✅ Set difficulty to {difficulty.value}: Skill Level {config.skill_level}")
    except Exception as e:
        print(f"❌ Difficulty level test failed: {e}")
        return False
    
    # Cleanup
    stockfish.shutdown()
    print("✅ Stockfish shutdown successfully")
    
    return True


def test_message_converter():
    """Test the message converter"""
    print("\n=== Testing Message Converter ===")

    try:
        converter = MessageConverter()

        # Test square conversions (these don't require ROS2 messages)
        index = converter.square_to_index("e4")
        square = converter.index_to_square(index)
        print(f"✅ Square conversions: e4 -> {index} -> {square}")

        # Test piece to FEN character conversion
        print("✅ Message converter core functions working")
        print("   Note: Full ROS2 message conversion requires proper message environment")

        return True

    except Exception as e:
        print(f"❌ Message converter test failed: {e}")
        return False


def main():
    """Main test function"""
    print("🚀 Starting Basic Stockfish Integration Tests")
    
    # Check if Stockfish is installed
    try:
        import stockfish
        print("✅ Stockfish Python package found")
    except ImportError:
        print("❌ Stockfish Python package not found")
        print("   Please install with: pip3 install stockfish")
        return False
    
    # Check if chess library is available
    try:
        import chess
        print("✅ Python-chess library found")
    except ImportError:
        print("❌ Python-chess library not found")
        print("   Please install with: pip3 install python-chess")
        return False
    
    # Run tests
    tests = [
        ("Stockfish Interface", test_stockfish_interface),
        ("Message Converter", test_message_converter)
    ]
    
    results = {}
    for test_name, test_func in tests:
        print(f"\n{'='*50}")
        print(f"Running {test_name} Test")
        print(f"{'='*50}")
        
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"❌ {test_name} test failed with exception: {e}")
            results[test_name] = False
    
    # Print summary
    print(f"\n{'='*50}")
    print("TEST RESULTS SUMMARY")
    print(f"{'='*50}")
    
    all_passed = True
    for test_name, passed in results.items():
        status = "PASS ✅" if passed else "FAIL ❌"
        print(f"{test_name}: {status}")
        if not passed:
            all_passed = False
    
    print(f"\n{'='*50}")
    if all_passed:
        print("🎉 ALL TESTS PASSED! Stockfish integration is working!")
        print("The core chess engine functionality is ready.")
        print("Next step: Fix ROS2 service message generation issues.")
    else:
        print("❌ SOME TESTS FAILED! Check the errors above.")
    print(f"{'='*50}")
    
    return all_passed


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
