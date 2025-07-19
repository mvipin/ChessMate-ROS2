#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessMate ROS2 Workspace Setup and Build Script
#
# This script sets up the complete ChessMate ROS2 development environment,
# installs dependencies, builds the workspace, and prepares for testing.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="humble"

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

print_header() {
    echo -e "\n${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}\n"
}

# Function to check if running on Raspberry Pi
check_raspberry_pi() {
    if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
        print_status $GREEN "✅ Running on Raspberry Pi"
        return 0
    else
        print_status $YELLOW "⚠️  Not running on Raspberry Pi - some features may not work"
        return 1
    fi
}

# Function to check ROS2 installation
check_ros2_installation() {
    print_header "Checking ROS2 Installation"
    
    if ! command -v ros2 &> /dev/null; then
        print_status $RED "❌ ROS2 not found"
        print_status $YELLOW "Installing ROS2 Humble..."
        install_ros2_humble
    else
        print_status $GREEN "✅ ROS2 found"
        
        # Check if it's the correct distribution
        if ros2 --version | grep -q "humble"; then
            print_status $GREEN "✅ ROS2 Humble confirmed"
        else
            print_status $YELLOW "⚠️  ROS2 version is not Humble"
        fi
    fi
}

# Function to install ROS2 Humble (for Raspberry Pi)
install_ros2_humble() {
    print_header "Installing ROS2 Humble"
    
    # Update system
    sudo apt update
    sudo apt upgrade -y
    
    # Install required packages
    sudo apt install -y \
        software-properties-common \
        curl \
        gnupg2 \
        lsb-release
    
    # Add ROS2 repository
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    
    # Update package list
    sudo apt update
    
    # Install ROS2 Humble
    sudo apt install -y ros-humble-desktop
    
    # Install additional tools and ROS2 packages
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-py \
        ros-humble-rosbag2-storage-default-plugins \
        ros-humble-rosbag2-transport \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-rviz2 \
        ros-humble-xacro
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    print_status $GREEN "✅ ROS2 Humble installation completed"
}

# Function to install system dependencies
install_system_dependencies() {
    print_header "Installing System Dependencies"
    
    # Update package list
    sudo apt update
    
    # Install required packages
    sudo apt install -y \
        python3-pip \
        python3-serial \
        python3-chess \
        stockfish \
        git \
        vim \
        htop \
        screen \
        udev
    
    # Install Python packages
    pip3 install --user \
        pyserial \
        python-chess \
        rclpy
    
    print_status $GREEN "✅ System dependencies installed"
}

# Function to setup USB permissions
setup_usb_permissions() {
    print_header "Setting Up USB Permissions"
    
    # Add user to dialout group
    sudo usermod -a -G dialout $USER
    
    # Create udev rules for ChessMate controllers
    sudo tee /etc/udev/rules.d/99-chessmate.rules > /dev/null << EOF
# ChessMate Controller USB Rules
# ChessBoard Controller (Pi Pico)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="chessmate_chessboard", GROUP="dialout", MODE="0666"

# Robot Controller (Pi Pico)  
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="chessmate_robot", GROUP="dialout", MODE="0666"

# Generic Pi Pico access
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", GROUP="dialout", MODE="0666"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_status $GREEN "✅ USB permissions configured"
    print_status $YELLOW "⚠️  You may need to log out and log back in for group changes to take effect"
}

# Function to install ROS2 dependencies
install_ros2_dependencies() {
    print_header "Installing ROS2 Dependencies"
    
    # Source ROS2
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    # Install workspace dependencies using rosdep
    cd "$WORKSPACE_DIR"
    rosdep install --from-paths src --ignore-src -r -y
    
    print_status $GREEN "✅ ROS2 dependencies installed"
}

# Function to build workspace
build_workspace() {
    print_header "Building ChessMate Workspace"
    
    cd "$WORKSPACE_DIR"
    
    # Source ROS2
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    # Clean previous build (optional)
    if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
        print_status $YELLOW "Cleaning previous build..."
        rm -rf build install log
    fi
    
    # Build workspace
    print_status $BLUE "Building workspace with colcon..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_status $GREEN "✅ Workspace built successfully"
    else
        print_status $RED "❌ Workspace build failed"
        exit 1
    fi
}

# Function to setup environment
setup_environment() {
    print_header "Setting Up Environment"
    
    # Create bashrc addition
    BASHRC_ADDITION="
# ChessMate ROS2 Environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $WORKSPACE_DIR/install/setup.bash
export CHESSMATE_WORKSPACE=$WORKSPACE_DIR
export ROS_DOMAIN_ID=42
"
    
    # Check if already added to bashrc
    if ! grep -q "ChessMate ROS2 Environment" ~/.bashrc; then
        echo "$BASHRC_ADDITION" >> ~/.bashrc
        print_status $GREEN "✅ Environment setup added to ~/.bashrc"
    else
        print_status $YELLOW "⚠️  Environment already configured in ~/.bashrc"
    fi
    
    # Create convenience scripts
    cat > "$WORKSPACE_DIR/source_workspace.sh" << 'EOF'
#!/bin/bash
# Source ChessMate workspace
source /opt/ros/humble/setup.bash
source $(dirname "$0")/install/setup.bash
export CHESSMATE_WORKSPACE=$(dirname "$0")
export ROS_DOMAIN_ID=42
echo "ChessMate workspace sourced"
EOF
    
    chmod +x "$WORKSPACE_DIR/source_workspace.sh"
    
    print_status $GREEN "✅ Environment configuration completed"
}

# Function to run basic tests
run_basic_tests() {
    print_header "Running Basic Tests"
    
    cd "$WORKSPACE_DIR"
    
    # Source workspace
    source install/setup.bash
    
    # Test 1: Check if packages are built
    if ros2 pkg list | grep -q chessmate; then
        print_status $GREEN "✅ ChessMate packages found"
    else
        print_status $RED "❌ ChessMate packages not found"
        return 1
    fi
    
    # Test 2: Check message types
    if ros2 interface list | grep -q chessmate_msgs; then
        print_status $GREEN "✅ ChessMate message types available"
    else
        print_status $RED "❌ ChessMate message types not found"
        return 1
    fi
    
    # Test 3: Check executables
    local executables=("chessboard_interface_node" "robot_interface_node" "game_management_node")
    for executable in "${executables[@]}"; do
        if ros2 pkg executables chessmate_hardware | grep -q "$executable"; then
            print_status $GREEN "✅ Executable $executable found"
        else
            print_status $YELLOW "⚠️  Executable $executable not found"
        fi
    done
    
    print_status $GREEN "✅ Basic tests completed"
}

# Function to create test scripts
create_test_scripts() {
    print_header "Creating Test Scripts"
    
    # Make test scripts executable
    chmod +x "$WORKSPACE_DIR/run_integration_tests.sh"
    
    # Create quick test script
    cat > "$WORKSPACE_DIR/quick_test.sh" << 'EOF'
#!/bin/bash
# Quick ChessMate test
cd "$(dirname "$0")"
source install/setup.bash
./run_integration_tests.sh quick
EOF
    
    chmod +x "$WORKSPACE_DIR/quick_test.sh"
    
    # Create controller test script
    cat > "$WORKSPACE_DIR/test_controllers.sh" << 'EOF'
#!/bin/bash
# Test ChessMate controllers
cd "$(dirname "$0")"
source install/setup.bash
./run_integration_tests.sh controllers
EOF
    
    chmod +x "$WORKSPACE_DIR/test_controllers.sh"
    
    print_status $GREEN "✅ Test scripts created"
}

# Function to display final instructions
display_final_instructions() {
    print_header "Setup Complete!"
    
    print_status $GREEN "🎉 ChessMate ROS2 workspace setup completed successfully!"
    
    echo -e "\n${BLUE}Next Steps:${NC}"
    echo "1. Log out and log back in (for USB permissions)"
    echo "2. Connect ChessBoard controller to USB (will appear as /dev/ttyACM0)"
    echo "3. Connect Robot controller to USB (will appear as /dev/ttyACM1)"
    echo "4. Upload controller firmware to Pi Picos"
    echo ""
    
    echo -e "${BLUE}Testing Commands:${NC}"
    echo "  ./scripts/test_controllers.sh     # Test controller communication"
    echo "  ./scripts/test_ros2_system.sh     # Test ROS2 system integration"
    echo ""
    
    echo -e "${BLUE}Source Workspace (Required):${NC}"
    if [ -d "install_arm" ]; then
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source install_arm/setup.bash"
    elif [ -d "install" ]; then
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source install/setup.bash"
    else
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source install/setup.bash  # (or install_arm/setup.bash on Pi)"
    fi
    echo ""

    echo -e "${BLUE}Manual Commands:${NC}"
    echo "  ros2 launch chessmate_hardware unified_hardware.launch.py"
    echo "  ros2 topic list               # View available topics"
    echo "  ros2 node list                # View running nodes"
    echo ""
    
    echo -e "${BLUE}Troubleshooting:${NC}"
    echo "  - Check USB connections: ls -la /dev/ttyACM*"
    echo "  - Check permissions: groups \$USER"
    echo "  - View logs: tail -f test_logs/integration_test_*.log"
    echo "  - Kill processes: pkill -f ros2"
}

# Main execution
main() {
    print_header "ChessMate ROS2 Workspace Setup"
    
    # Check system
    check_raspberry_pi
    
    # Install and setup
    check_ros2_installation
    install_system_dependencies
    setup_usb_permissions
    install_ros2_dependencies
    build_workspace
    setup_environment
    create_test_scripts
    
    # Test basic functionality
    run_basic_tests
    
    # Display final instructions
    display_final_instructions
    
    print_status $GREEN "✅ Setup completed successfully!"
}

# Help function
show_help() {
    echo "ChessMate ROS2 Workspace Setup Script"
    echo ""
    echo "This script sets up the complete ChessMate ROS2 development environment."
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show this help message"
    echo "  --skip-ros2    Skip ROS2 installation (if already installed)"
    echo "  --build-only   Only build workspace (skip system setup)"
    echo ""
    echo "What this script does:"
    echo "  1. Checks/installs ROS2 Humble"
    echo "  2. Installs system dependencies (Python, Stockfish, etc.)"
    echo "  3. Sets up USB permissions for Pi Pico controllers"
    echo "  4. Installs ROS2 package dependencies"
    echo "  5. Builds the ChessMate workspace"
    echo "  6. Configures environment variables"
    echo "  7. Creates test scripts"
    echo "  8. Runs basic validation tests"
}

# Parse command line arguments
SKIP_ROS2=false
BUILD_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --skip-ros2)
            SKIP_ROS2=true
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Run appropriate setup
if [ "$BUILD_ONLY" = true ]; then
    print_header "Build Only Mode"
    source /opt/ros/$ROS_DISTRO/setup.bash
    build_workspace
    run_basic_tests
else
    main
fi
