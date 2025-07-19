#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessBoard Controller Test Script

echo "🏁 ChessBoard Controller Test"
echo "============================="

# Check if Python script exists
if [ ! -f "test_chessboard_controller.py" ]; then
    echo "❌ test_chessboard_controller.py not found"
    exit 1
fi

# Check if USB device exists
if [ ! -e "/dev/ttyACM0" ]; then
    echo "❌ ChessBoard controller not found at /dev/ttyACM0"
    echo "   Make sure the Pi Pico is connected via USB"
    exit 1
fi

# Check permissions
if [ ! -r "/dev/ttyACM0" ] || [ ! -w "/dev/ttyACM0" ]; then
    echo "⚠️  Permission issue with /dev/ttyACM0"
    echo "   You may need to add your user to the dialout group:"
    echo "   sudo usermod -a -G dialout $USER"
    echo "   Then log out and log back in"
fi

# Run the test
echo "🚀 Starting ChessBoard controller test..."
python3 test_chessboard_controller.py "$@"
