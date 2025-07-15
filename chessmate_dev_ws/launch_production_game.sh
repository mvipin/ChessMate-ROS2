#!/bin/bash

# Launch Production ChessMate Game
echo "🎮 ChessMate Production Game Launch"
echo "=================================="

cd "$(dirname "$0")"

# Parse command line arguments
HARDWARE_MODE="real"  # Default to real mode

while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            HARDWARE_MODE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--mode <mock|real>]"
            echo ""
            echo "Options:"
            echo "  --mode <mock|real>   Set hardware mode (default: real)"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                   # Run in real mode (default)"
            echo "  $0 --mode real       # Run with real hardware"
            echo "  $0 --mode mock       # Run in mock mode (no sensors)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate mode parameter
if [[ "$HARDWARE_MODE" != "mock" && "$HARDWARE_MODE" != "real" ]]; then
    echo "❌ Error: Invalid mode '$HARDWARE_MODE'. Must be 'mock' or 'real'"
    exit 1
fi

echo "Hardware mode: $HARDWARE_MODE"
echo ""

# Build everything
#echo "🔨 Building ChessMate system..."
#colcon build --packages-select chessmate_engine chessmate_hardware chessmate_msgs --build-base build_arm --install-base install_arm --symlink-install

#if [ $? -ne 0 ]; then
#    echo "❌ Build failed!"
#    exit 1
#fi

# Source workspace
source setup_ros2_fixed.bash

# Check for controller availability
echo "🔍 Checking for Pico controllers..."

if [ ! -e "/dev/ttyACM0" ] || [ ! -e "/dev/ttyACM1" ]; then
    echo "❌ Error: Pico controllers not found:"
    [ ! -e "/dev/ttyACM0" ] && echo "   ChessBoard controller missing at /dev/ttyACM0"
    [ ! -e "/dev/ttyACM1" ] && echo "   Robot controller missing at /dev/ttyACM1"
    echo ""
    echo "💡 Please connect the Pico controllers and try again"
    exit 1
else
    echo "✅ Pico controllers found:"
    echo "   ChessBoard controller: /dev/ttyACM0"
    echo "   Robot controller: /dev/ttyACM1"
    echo "   Mode: $HARDWARE_MODE"
fi

# Cleanup function
cleanup() {
    echo "🧹 Cleaning up production components..."
    pkill -f "topic_chess_engine_server" 2>/dev/null
    pkill -f "topic_arduino_communication" 2>/dev/null
    pkill -f "topic_game_management" 2>/dev/null
    wait
    exit
}
trap cleanup EXIT INT TERM

echo "🚀 Starting ChessMate production system..."

# Start chess engine
echo "1️⃣ Starting Stockfish chess engine..."
ros2 run chessmate_engine topic_chess_engine_server &
ENGINE_PID=$!

# Start Arduino communication with real hardware
echo "2️⃣ Starting Arduino communication (${HARDWARE_MODE} mode)..."
ros2 run chessmate_hardware topic_arduino_communication \
    --ros-args \
    --param hardware_mode:=${HARDWARE_MODE} \
    --param chessboard_port:=/dev/ttyACM0 \
    --param robot_port:=/dev/ttyACM1 &
ARDUINO_PID=$!

# Start game management
echo "3️⃣ Starting game management..."
ros2 run chessmate_engine topic_game_management \
    --ros-args \
    --param hardware_mode:=${HARDWARE_MODE} \
    --param auto_start:=true \
    --param skill_level:=19 \
    --param time_limit:=3.0 &
GAME_PID=$!

# Wait for components to initialize
echo "⏳ Waiting for components to initialize..."
sleep 10

# Check if all components are running
components_ok=true

if ! kill -0 $ENGINE_PID 2>/dev/null; then
    echo "❌ Chess engine failed to start"
    components_ok=false
fi

if ! kill -0 $ARDUINO_PID 2>/dev/null; then
    echo "❌ Arduino communication failed to start"
    components_ok=false
fi

if ! kill -0 $GAME_PID 2>/dev/null; then
    echo "❌ Game management failed to start"
    components_ok=false
fi

if [ "$components_ok" = false ]; then
    echo "❌ Component startup failed"
    exit 1
fi

echo ""
echo "✅ All components running successfully:"
echo "   Chess Engine PID: $ENGINE_PID"
echo "   Arduino Comm PID: $ARDUINO_PID"
echo "   Game Manager PID: $GAME_PID"
echo ""
echo "🎮 ChessMate is ready for play!"
echo ""
echo "📋 Game Status:"
echo "   - Hardware mode: ${HARDWARE_MODE}"
echo "   - ChessBoard: /dev/ttyACM0"
echo "   - Robot: /dev/ttyACM1"
echo "   - Skill level: 5"
echo ""
echo "🎯 To monitor the game:"
echo "   ros2 topic echo /game/state"
echo "   ros2 topic echo /chessboard/moves"
echo ""
echo "🛑 Press Ctrl+C to stop the game"
echo ""

# Keep running until interrupted
while true; do
    sleep 1
    
    # Check if any component died
    if ! kill -0 $ENGINE_PID 2>/dev/null; then
        echo "❌ Chess engine died!"
        break
    fi
    
    if ! kill -0 $ARDUINO_PID 2>/dev/null; then
        echo "❌ Arduino communication died!"
        break
    fi
    
    if ! kill -0 $GAME_PID 2>/dev/null; then
        echo "❌ Game management died!"
        break
    fi
done

echo "🏁 ChessMate production game ended"
