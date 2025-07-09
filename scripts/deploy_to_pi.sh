#!/bin/bash
# ChessMate Deployment Script for Raspberry Pi - Character Protocol Only
# Usage: ./deploy_to_pi.sh [pi_hostname_or_ip]

set -e

PI_HOST=${1:-"chessmate"}
PI_USER=${2:-"pi"}

echo "🚀 ChessMate Deployment to Raspberry Pi (Character Protocol)"
echo "Target: $PI_USER@$PI_HOST"
echo "=========================================================="

# Step 1: Create deployment package
echo "📦 Creating deployment package..."
cd ~/ChessMate
tar -czf chessmate_unified_deployment.tar.gz \
    chessmate_dev_ws/src/chessmate_msgs \
    chessmate_dev_ws/src/chessmate_hardware \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log'

echo "✅ Package created: chessmate_unified_deployment.tar.gz"

# Step 2: Transfer to Pi
echo "📤 Transferring to Raspberry Pi..."
scp chessmate_unified_deployment.tar.gz $PI_USER@$PI_HOST:~/

# Step 3: Remote setup
echo "🔧 Setting up on Raspberry Pi..."
ssh $PI_USER@$PI_HOST << 'EOF'
    set -e
    
    echo "📂 Extracting deployment package..."
    cd ~
    tar -xzf chessmate_unified_deployment.tar.gz
    
    echo "🏗️  Setting up workspace..."
    mkdir -p ~/chessmate_dev_ws
    cd ~/chessmate_dev_ws
    
    echo "📚 Sourcing ROS 2..."
    source /opt/ros/humble/setup.bash
    
    echo "🔨 Building packages..."
    colcon build --packages-select chessmate_msgs
    colcon build --packages-select chessmate_hardware
    
    echo "✅ Build complete!"
    
    echo "🧪 Running sanity tests..."
    source install/setup.bash
    
    # Test 1: Platform detection
    echo "Test 1: Platform Detection"
    python3 -c "
from src.chessmate_hardware.chessmate_hardware.config_loader import get_config
config = get_config()
print(f'Platform: {config.get_platform_type()}')
print(f'Mock mode: {config.is_mock_mode()}')
"
    
    # Test 2: GPIO check
    echo "Test 2: GPIO Access"
    python3 -c "
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    print('✅ GPIO access working')
    GPIO.cleanup()
except Exception as e:
    print(f'❌ GPIO access failed: {e}')
"
    
    # Test 3: Integration validation
    echo "Test 3: Integration Validation"
    python3 src/chessmate_hardware/scripts/validate_integration.py
    
    echo "🎉 Deployment and sanity tests complete!"
    echo ""
    echo "Next steps:"
    echo "1. Connect Arduino controllers to USB ports"
    echo "2. Run: ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real"
    echo "3. Test with: ros2 run chessmate_hardware unified_hardware_test comprehensive"
EOF

echo "🎯 Deployment complete!"
echo ""
echo "To connect and test:"
echo "ssh $PI_USER@$PI_HOST"
echo "cd ~/chessmate_dev_ws && source install/setup.bash"
echo "ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real"
