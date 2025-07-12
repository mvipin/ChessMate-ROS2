#!/bin/bash
# ChessMate Complete System Test Script

echo "🏁 ChessMate System Test"
echo "========================"

# Function to check if a device exists
check_device() {
    local device=$1
    local name=$2
    
    if [ ! -e "$device" ]; then
        echo "❌ $name not found at $device"
        return 1
    else
        echo "✅ $name found at $device"
        return 0
    fi
}

# Function to check permissions
check_permissions() {
    local device=$1
    local name=$2
    
    if [ ! -r "$device" ] || [ ! -w "$device" ]; then
        echo "⚠️  Permission issue with $device ($name)"
        echo "   You may need to add your user to the dialout group:"
        echo "   sudo usermod -a -G dialout $USER"
        echo "   Then log out and log back in"
        return 1
    else
        echo "✅ $name permissions OK"
        return 0
    fi
}

# Check for required devices
echo "🔍 Checking for controllers..."
check_device "/dev/ttyACM0" "ChessBoard Controller" || exit 1
check_device "/dev/ttyACM1" "Robot Controller" || echo "⚠️  Robot Controller not found (optional for ChessBoard-only tests)"

# Check permissions
echo ""
echo "🔐 Checking permissions..."
check_permissions "/dev/ttyACM0" "ChessBoard Controller"
if [ -e "/dev/ttyACM1" ]; then
    check_permissions "/dev/ttyACM1" "Robot Controller"
fi

# Check for Python scripts (updated paths for scripts directory)
echo ""
echo "📋 Checking for test scripts..."
for script in "../ChessBoard/test_chessboard_controller.py" "../Robot/test_robot_controller.py" "test_complete_game.py"; do
    if [ -f "$script" ]; then
        echo "✅ $script found"
    else
        echo "❌ $script not found"
        exit 1
    fi
done

echo ""
echo "🚀 What would you like to test?"
echo "1. ChessBoard Controller only"
echo "2. Robot Controller only"
echo "3. Complete game simulation"
echo "4. Interactive ChessBoard test"
echo "5. Interactive Robot test"
echo "6. All tests (automated)"

read -p "Enter your choice (1-6): " choice

case $choice in
    1)
        echo "🎯 Testing ChessBoard Controller..."
        cd ../ChessBoard
        python3 test_chessboard_controller.py
        cd ../scripts
        ;;
    2)
        echo "🤖 Testing Robot Controller..."
        if [ ! -e "/dev/ttyACM1" ]; then
            echo "❌ Robot Controller not found at /dev/ttyACM1"
            exit 1
        fi
        cd ../Robot
        python3 test_robot_controller.py
        cd ../scripts
        ;;
    3)
        echo "🏁 Running complete game simulation..."
        if [ ! -e "/dev/ttyACM1" ]; then
            echo "❌ Robot Controller not found at /dev/ttyACM1"
            echo "   Complete game simulation requires both controllers"
            exit 1
        fi
        python3 test_complete_game.py full
        ;;
    4)
        echo "🎮 Starting interactive ChessBoard test..."
        cd ../ChessBoard
        python3 test_chessboard_controller.py interactive
        cd ../scripts
        ;;
    5)
        echo "🎮 Starting interactive Robot test..."
        if [ ! -e "/dev/ttyACM1" ]; then
            echo "❌ Robot Controller not found at /dev/ttyACM1"
            exit 1
        fi
        cd ../Robot
        python3 test_robot_controller.py interactive
        cd ../scripts
        ;;
    6)
        echo "🔄 Running all automated tests..."
        
        echo ""
        echo "📋 Test 1: ChessBoard Controller"
        cd ../ChessBoard
        python3 test_chessboard_controller.py
        cd ../scripts
        
        if [ -e "/dev/ttyACM1" ]; then
            echo ""
            echo "📋 Test 2: Robot Controller"
            cd ../Robot
            python3 test_robot_controller.py
            cd ../scripts
            
            echo ""
            echo "📋 Test 3: Complete Game Simulation"
            python3 test_complete_game.py full
        else
            echo ""
            echo "⚠️  Skipping Robot and Complete Game tests (Robot Controller not found)"
        fi
        
        echo ""
        echo "🎉 All available tests completed!"
        ;;
    *)
        echo "❌ Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "✅ Test session complete!"
