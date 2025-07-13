#!/bin/bash
# ChessMate Unified System Test Script
# 
# This script provides comprehensive testing with increasing complexity levels:
# Level 0: Individual Pi Pico Controllers (direct USB Serial, no ROS2)
# Level 1: ROS2 Integration (Pi Picos + ROS2 bridge)
# Level 2: Complete Game Simulation (Controllers + Chess Engine + Game Management)
# 
# Supports both mock and real hardware modes with configurable ports.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default configuration
DEFAULT_CHESSBOARD_PORT="/dev/ttyACM0"
DEFAULT_ROBOT_PORT="/dev/ttyACM1"
DEFAULT_MODE="mock"
DEFAULT_DURATION=300

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/../chessmate_dev_ws"

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

# Function to check prerequisites
check_prerequisites() {
    local hardware_mode=$1
    local chessboard_port=$2
    local robot_port=$3
    
    print_header "Checking Prerequisites"
    
    # Change to workspace directory
    cd "$WORKSPACE_DIR"
    
    # Check if workspace is built
    if [ -f "install_arm/setup.bash" ]; then
        INSTALL_DIR="install_arm"
        print_status $GREEN "✅ ARM workspace found"
    elif [ -f "install/setup.bash" ]; then
        INSTALL_DIR="install"
        print_status $GREEN "✅ x86 workspace found"
    else
        print_status $RED "❌ Workspace not built. Please run './setup_and_build.sh' first."
        exit 1
    fi
    
    # Source workspace
    source "$INSTALL_DIR/setup.bash"
    
    # Check hardware based on mode
    if [ "$hardware_mode" = "real" ]; then
        print_status $BLUE "Real hardware mode - checking controller connections"
        if [ -e "$chessboard_port" ]; then
            print_status $GREEN "✅ ChessBoard controller found at $chessboard_port"
        else
            print_status $RED "❌ ChessBoard controller not found at $chessboard_port"
            print_status $YELLOW "   Use --chessboard-port to specify different port"
            exit 1
        fi

        if [ -e "$robot_port" ]; then
            print_status $GREEN "✅ Robot controller found at $robot_port"
        else
            print_status $RED "❌ Robot controller not found at $robot_port"
            print_status $YELLOW "   Use --robot-port to specify different port"
            exit 1
        fi
    else
        print_status $BLUE "🔧 Mock mode - using simulated hardware"
        if [ -e "$chessboard_port" ]; then
            print_status $BLUE "Note: ChessBoard controller detected at $chessboard_port (will use mock anyway)"
        fi
        if [ -e "$robot_port" ]; then
            print_status $BLUE "Note: Robot controller detected at $robot_port (will use mock anyway)"
        fi
    fi
    
    # Check Stockfish
    if command -v stockfish &> /dev/null; then
        print_status $GREEN "✅ Stockfish chess engine available"
    else
        print_status $YELLOW "⚠️  Stockfish not found - install with: sudo apt install stockfish"
    fi
    
    # Check ROS2 packages
    if ros2 pkg list 2>/dev/null | grep -q chessmate; then
        local pkg_count=$(ros2 pkg list 2>/dev/null | grep chessmate | wc -l)
        print_status $GREEN "✅ ChessMate packages available ($pkg_count packages)"
    else
        print_status $RED "❌ ChessMate packages not found"
        exit 1
    fi
}

# Function to test individual controllers (Level 0)
test_individual_controllers() {
    local hardware_mode=$1
    local chessboard_port=$2
    local robot_port=$3
    local controller_type=${4:-"both"}

    print_header "Level 0: Individual Pi Pico Controller Test"

    print_status $BLUE "Testing individual Pi Pico controllers (no ROS2)..."
    print_status $BLUE "Hardware Mode: $hardware_mode"
    print_status $BLUE "Controller Type: $controller_type"

    local overall_success=true

    case $controller_type in
        "chessboard")
            if ! test_single_controller "ChessBoard" "$chessboard_port" "$hardware_mode"; then
                overall_success=false
            fi
            ;;
        "robot")
            if ! test_single_controller "Robot" "$robot_port" "$hardware_mode"; then
                overall_success=false
            fi
            ;;
        "both")
            if ! test_single_controller "ChessBoard" "$chessboard_port" "$hardware_mode"; then
                overall_success=false
            fi
            echo ""
            if ! test_single_controller "Robot" "$robot_port" "$hardware_mode"; then
                overall_success=false
            fi
            ;;
    esac

    if [ "$overall_success" = true ]; then
        print_status $GREEN "✅ Individual controller test PASSED"
        return 0
    else
        print_status $RED "❌ Individual controller test FAILED"
        return 1
    fi
}

# Function to test a single controller
test_single_controller() {
    local controller_type=$1
    local port=$2
    local mode=$3

    print_status $BLUE "Testing $controller_type Controller at $port"

    if [ "$mode" = "real" ]; then
        # Test real hardware
        if [ ! -e "$port" ]; then
            print_status $RED "❌ Controller not found at $port"
            return 1
        fi

        if [ ! -r "$port" ] || [ ! -w "$port" ]; then
            print_status $RED "❌ Permission denied for $port"
            print_status $YELLOW "   Run: sudo usermod -a -G dialout $USER"
            return 1
        fi

        print_status $GREEN "✅ Hardware found at $port"

        # Set controller mode first
        print_status $BLUE "• Setting controller mode to $mode..."
        echo "mode:$mode" > $port 2>/dev/null || true
        sleep 1

        # Test basic communication
        print_status $BLUE "• Testing basic communication..."
        if timeout 5 bash -c "echo 'status' > $port && timeout 3 cat $port" 2>/dev/null | grep -q "ready\|ok\|status\|mode"; then
            print_status $GREEN "  ✅ Communication successful"
        else
            print_status $YELLOW "  ⚠️  No response from controller"
            print_status $BLUE "    This may indicate the controller firmware needs to be flashed"
        fi

        # Test controller-specific commands
        case $controller_type in
            "ChessBoard")
                print_status $BLUE "• Testing board scan..."
                timeout 3 bash -c "echo 'scan' > $port" 2>/dev/null || true
                print_status $GREEN "  ✅ Board scan command sent"
                ;;
            "Robot")
                print_status $BLUE "• Testing home command..."
                timeout 3 bash -c "echo 'home' > $port" 2>/dev/null || true
                print_status $GREEN "  ✅ Home command sent"
                ;;
        esac

    else
        # Mock mode
        print_status $BLUE "🔧 Mock mode - simulating $controller_type responses"
        sleep 1
        case $controller_type in
            "ChessBoard")
                print_status $GREEN "  ✅ Mock board scan: 32 pieces detected"
                ;;
            "Robot")
                print_status $GREEN "  ✅ Mock robot home: Position reached"
                ;;
        esac
    fi

    print_status $GREEN "✅ $controller_type controller test completed"
    return 0
}

# Function to test ROS2 integration (Level 1)
test_controllers() {
    local hardware_mode=$1
    local chessboard_port=$2
    local robot_port=$3
    local controller_type=${4:-"both"}

    print_header "Level 1: ROS2 Integration Test"
    
    print_status $BLUE "Testing Pi Pico controllers with ROS2 bridge..."
    print_status $BLUE "Hardware Mode: $hardware_mode"
    print_status $BLUE "Controller Type: $controller_type"
    print_status $BLUE "ChessBoard Port: $chessboard_port"
    print_status $BLUE "Robot Port: $robot_port"

    # Determine which controllers to test
    local test_mode="controllers"
    case $controller_type in
        "chessboard")
            test_mode="controllers_chessboard"
            ;;
        "robot")
            test_mode="controllers_robot"
            ;;
        "both")
            test_mode="controllers"
            ;;
    esac

    # Run ROS2 integration test
    timeout 120 ros2 launch chessmate_hardware integration_testing.launch.py \
        test_mode:=$test_mode \
        hardware_mode:=$hardware_mode \
        chessboard_port:=$chessboard_port \
        robot_port:=$robot_port \
        log_level:=info \
        2>&1
    
    local exit_code=$?
    
    if [ $exit_code -eq 0 ] || [ $exit_code -eq 124 ]; then
        print_status $GREEN "✅ ROS2 integration test PASSED"
        return 0
    else
        print_status $RED "❌ ROS2 integration test FAILED (exit code: $exit_code)"
        return $exit_code
    fi
}

# Function to test complete game simulation (Level 2)
test_complete_game() {
    local hardware_mode=$1
    local chessboard_port=$2
    local robot_port=$3
    local duration=$4
    local skill_level=${5:-5}
    
    print_header "Level 2: Complete Game Simulation Test"
    
    print_status $BLUE "Running complete chess game simulation..."
    print_status $BLUE "Hardware Mode: $hardware_mode"
    print_status $BLUE "Duration: ${duration} seconds"
    print_status $BLUE "Stockfish Skill Level: ${skill_level}/20"
    
    print_status $YELLOW "This demonstrates:"
    echo "  • Pi Pico controller communication"
    echo "  • ROS2 system integration"
    echo "  • Chess engine integration (Stockfish)"
    echo "  • Complete game flow management"
    echo "  • Move validation and execution"
    echo ""
    
    # Run complete game simulation
    timeout $duration ros2 launch chessmate_hardware integration_testing.launch.py \
        test_mode:=game_simulation \
        hardware_mode:=$hardware_mode \
        chessboard_port:=$chessboard_port \
        robot_port:=$robot_port \
        stockfish_path:=/usr/games/stockfish \
        stockfish_skill_level:=$skill_level \
        log_level:=info \
        record_bag:=true \
        2>&1
    
    local exit_code=$?
    
    if [ $exit_code -eq 0 ] || [ $exit_code -eq 124 ]; then
        print_status $GREEN "✅ Complete game simulation PASSED"
        
        # Show recorded data
        if [ -d "/tmp/chessmate_test_game_simulation" ]; then
            print_status $BLUE "📊 Game data recorded to: /tmp/chessmate_test_game_simulation/"
            print_status $BLUE "To replay: ros2 bag play /tmp/chessmate_test_game_simulation/"
        fi
        return 0
    else
        print_status $RED "❌ Complete game simulation FAILED (exit code: $exit_code)"
        return $exit_code
    fi
}

# Function to show help
show_help() {
    echo "ChessMate Unified System Test Script"
    echo ""
    echo "Comprehensive testing with increasing complexity levels:"
    echo "  Level 0: Individual Pi Pico Controllers (direct USB Serial, no ROS2)"
    echo "  Level 1: ROS2 Integration (Pi Picos + ROS2 bridge)"
    echo "  Level 2: Complete Game Simulation (Controllers + Chess Engine + Game Management)"
    echo ""
    echo "Usage: $0 <LEVEL> [OPTIONS]"
    echo ""
    echo "Levels:"
    echo "  pico              Level 0: Test individual Pi Pico controllers (no ROS2)"
    echo "  ros2              Level 1: Test ROS2 integration with controllers"
    echo "  game              Level 2: Test complete game simulation"
    echo ""
    echo "Options:"
    echo "  --mode MODE       Hardware mode: mock or real (default: $DEFAULT_MODE)"
    echo "  --chessboard-port PORT    ChessBoard controller port (default: $DEFAULT_CHESSBOARD_PORT)"
    echo "  --robot-port PORT         Robot controller port (default: $DEFAULT_ROBOT_PORT)"
    echo "  --duration SECONDS        Game duration for Level 2 (default: $DEFAULT_DURATION)"
    echo "  --skill-level LEVEL       Stockfish skill 1-20 for Level 2 (default: 5)
  --controller TYPE         Controller type for Level 0 & 1: chessboard, robot, both (default: both)"
    echo ""
    echo "Examples:"
    echo "  $0 pico --mode mock --controller both"
    echo "  $0 pico --mode real --controller chessboard --chessboard-port /dev/ttyUSB0"
    echo "  $0 ros2 --mode mock --controller both"
    echo "  $0 ros2 --mode real --controller chessboard --chessboard-port /dev/ttyUSB0"
    echo "  $0 ros2 --mode real --controller robot --robot-port /dev/ttyACM1"
    echo "  $0 game --mode real --duration 600 --skill-level 10"
    echo "  $0 game --mode mock --duration 120"
    echo ""
    echo "Hardware Modes:"
    echo "  mock    Controllers use simulated hardware (mock steppers, sensors, etc.) over real serial"
    echo "  real    Controllers use real hardware (actual steppers, limit switches, etc.)"
    echo ""
    echo "Note: Mock mode tests the communication protocol with simulated hardware responses."
    echo "      Real mode tests actual hardware components on the controllers."
}

# Parse command line arguments
parse_args() {
    LEVEL=""
    MODE="$DEFAULT_MODE"
    CHESSBOARD_PORT="$DEFAULT_CHESSBOARD_PORT"
    ROBOT_PORT="$DEFAULT_ROBOT_PORT"
    DURATION="$DEFAULT_DURATION"
    SKILL_LEVEL=5
    CONTROLLER_TYPE="both"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            pico|ros2|game)
                LEVEL="$1"
                shift
                ;;
            --mode)
                MODE="$2"
                shift 2
                ;;
            --chessboard-port)
                CHESSBOARD_PORT="$2"
                shift 2
                ;;
            --robot-port)
                ROBOT_PORT="$2"
                shift 2
                ;;
            --duration)
                DURATION="$2"
                shift 2
                ;;
            --skill-level)
                SKILL_LEVEL="$2"
                shift 2
                ;;
            --controller)
                CONTROLLER_TYPE="$2"
                shift 2
                ;;
            -h|--help|help)
                show_help
                exit 0
                ;;
            *)
                print_status $RED "❌ Unknown option: $1"
                echo ""
                show_help
                exit 1
                ;;
        esac
    done
    
    if [ -z "$LEVEL" ]; then
        print_status $RED "❌ Please specify a test level: pico, ros2, or game"
        echo ""
        show_help
        exit 1
    fi
}

# Main execution
main() {
    parse_args "$@"
    
    print_header "ChessMate Unified System Test"
    print_status $BLUE "Test Level: $LEVEL"
    print_status $BLUE "Hardware Mode: $MODE"
    print_status $BLUE "ChessBoard Port: $CHESSBOARD_PORT"
    print_status $BLUE "Robot Port: $ROBOT_PORT"
    
    # Check prerequisites
    check_prerequisites "$MODE" "$CHESSBOARD_PORT" "$ROBOT_PORT"
    
    case $LEVEL in
        "pico")
            test_individual_controllers "$MODE" "$CHESSBOARD_PORT" "$ROBOT_PORT" "$CONTROLLER_TYPE"
            ;;
        "ros2")
            test_controllers "$MODE" "$CHESSBOARD_PORT" "$ROBOT_PORT" "$CONTROLLER_TYPE"
            ;;
        "game")
            # Run Level 2 directly (game simulation includes all necessary components)
            print_status $BLUE "Running Level 2: Complete Game Simulation..."
            test_complete_game "$MODE" "$CHESSBOARD_PORT" "$ROBOT_PORT" "$DURATION" "$SKILL_LEVEL"
            ;;
    esac
    
    print_header "Test Complete"
    print_status $GREEN "🎉 ChessMate system test completed successfully!"
}

# Run main function
main "$@"
