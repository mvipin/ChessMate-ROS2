#!/bin/bash
# Source ChessMate ARM Environment  
# Usage: source ./source_arm.sh

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "🚀 Setting up ChessMate ARM Environment"
echo "======================================="

# Clear old environment variables that might conflict
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source ChessMate ARM build
if [ -f "install_arm/setup.bash" ]; then
    source install_arm/setup.bash
    echo "✅ ChessMate ARM environment ready"
    echo "📁 Using install: install_arm/"
else
    echo "❌ ARM build not found. Run ./build_arm.sh first"
    return 1
fi

# Set environment
export ROS_DOMAIN_ID=0
export CHESSMATE_PLATFORM="raspberry_pi"

echo ""
echo "🎯 Available commands:"
echo "   ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real"
echo "   ros2 run chessmate_hardware test_arduino_serial --port /dev/ttyACM0"
echo "   ros2 run chessmate_hardware test_ros2_hardware"
