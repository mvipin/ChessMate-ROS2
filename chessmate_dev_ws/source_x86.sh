#!/bin/bash
# Source ChessMate x86 Environment
# Usage: source ./source_x86.sh

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "🚀 Setting up ChessMate x86 Environment"
echo "======================================="

# Clear old environment variables that might conflict
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source ChessMate x86 build
if [ -f "install_x86/setup.bash" ]; then
    source install_x86/setup.bash
    echo "✅ ChessMate x86 environment ready"
    echo "📁 Using install: install_x86/"
else
    echo "❌ x86 build not found. Run ./build_x86.sh first"
    return 1
fi

# Set environment
export ROS_DOMAIN_ID=0
export CHESSMATE_PLATFORM="linux_host"

echo ""
echo "🎯 Available commands:"
echo "   ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=mock"
echo "   ros2 run chessmate_hardware test_arduino_serial"
echo "   ros2 run chessmate_hardware unified_hardware_test"
