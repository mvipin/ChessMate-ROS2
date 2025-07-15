#!/bin/bash
# Fixed ROS2 environment setup

# Source ROS2 base
source /opt/ros/humble/setup.bash

# Set missing environment variables if needed
export ROS_VERSION=2
export ROS_DISTRO=humble

# Set default RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source workspace
if [ -f "install_arm/setup.bash" ]; then
    source install_arm/setup.bash
fi

echo "✅ ROS2 environment fixed and ready"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION" 
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
