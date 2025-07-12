#!/bin/bash
# Robot Controller Test Script

echo "🤖 Robot Controller Test"
echo "========================"
echo "ℹ️  Using merged Robot.ino with USB communication support"

# Check if Python script exists
if [ ! -f "test_robot_controller.py" ]; then
    echo "❌ test_robot_controller.py not found"
    exit 1
fi

# Check if USB device exists
if [ ! -e "/dev/ttyACM1" ]; then
    echo "❌ Robot controller not found at /dev/ttyACM1"
    echo "   Make sure the Robot Pi Pico is connected via USB"
    echo "   You may need to adjust the device path if using a different port"
    exit 1
fi

# Check permissions
if [ ! -r "/dev/ttyACM1" ] || [ ! -w "/dev/ttyACM1" ]; then
    echo "⚠️  Permission issue with /dev/ttyACM1"
    echo "   You may need to add your user to the dialout group:"
    echo "   sudo usermod -a -G dialout $USER"
    echo "   Then log out and log back in"
fi

# Run the test
echo "🚀 Starting Robot controller test..."
python3 test_robot_controller.py "$@"
