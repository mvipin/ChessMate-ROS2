#!/bin/bash
# Build ChessMate for ARM (Raspberry Pi) Architecture
# Usage: ./build_arm.sh [package_name]

set -e

cd "$(dirname "$0")"

echo "🏗️  Building ChessMate for ARM Architecture"
echo "==========================================="

# Setup environment
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

# Build arguments with comprehensive Python configuration
BUILD_ARGS=(
    --build-base build_arm
    --install-base install_arm
    --cmake-args
        -DPython3_EXECUTABLE=/usr/bin/python3
        -DPYTHON_EXECUTABLE=/usr/bin/python3
        -DPython3_FIND_STRATEGY=LOCATION
        -DPython3_ROOT_DIR=/usr
        -DCMAKE_BUILD_TYPE=Release
)

# Add package selection if specified
if [ $# -gt 0 ]; then
    BUILD_ARGS+=(--packages-select "$@")
    echo "Building packages: $*"
else
    echo "Building all packages"
fi

# Set separate log directory for ARM
export COLCON_LOG_PATH="$(pwd)/log_arm"
mkdir -p "$COLCON_LOG_PATH"

# Build
echo "Building..."
colcon build "${BUILD_ARGS[@]}"

echo ""
echo "✅ ARM build complete!"
echo "📁 Build artifacts in: build_arm/"
echo "📁 Install artifacts in: install_arm/"
echo "📁 Logs in: log_arm/"
echo ""
echo "🚀 To use:"
echo "   source install_arm/setup.bash"
echo "   # or run: ./source_arm.sh"
