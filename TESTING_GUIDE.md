# ChessMate Testing Guide - Character Protocol Only

This guide provides a clear testing workflow for the simplified ChessMate system using character-based protocol only.

## 📁 **Organized Script Structure**

### **Arduino Directory** (`arduino/ControllerArm/chessmate_arduino_stub/`)
- `chessmate_arduino_stub.ino` - Arduino sketch (character protocol only)
- `README.md` - Arduino setup instructions

### **ROS 2 Testing Scripts** (`chessmate_dev_ws/src/chessmate_hardware/chessmate_hardware/`)
- `test_arduino_serial.py` - Direct Arduino serial communication test
- `test_ros2_hardware.py` - ROS 2 hardware integration test  
- `unified_hardware_test.py` - Comprehensive system test
- `validate_integration.py` - Codebase integration validation

### **Deployment Scripts** (`chessmate_dev_ws/scripts/`)
- `deploy_to_pi.sh` - Deploy codebase to Raspberry Pi

## 🧪 **Testing Workflow**

### **Step 1: Arduino Setup**
```bash
# 1. Upload Arduino sketch
# Open arduino/ControllerArm/chessmate_arduino_stub/chessmate_arduino_stub.ino
# Upload to Arduino using Arduino IDE

# 2. Test direct serial communication
ros2 run chessmate_hardware test_arduino_serial --port /dev/ttyACM0
```

### **Step 2: ROS 2 Integration Testing**
```bash
# 1. Start the hardware interface
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real

# 2. In another terminal, test ROS 2 integration
ros2 run chessmate_hardware test_ros2_hardware

# 3. Run comprehensive tests
ros2 run chessmate_hardware unified_hardware_test comprehensive
```

### **Step 3: System Validation**
```bash
# Validate the entire integration
python3 src/chessmate_hardware/scripts/validate_integration.py
```

## 🎯 **Script Purposes (No Overlap)**

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `test_arduino_serial` | Direct Arduino communication | First test after Arduino upload |
| `test_ros2_hardware` | ROS 2 topic/service integration | After hardware interface is running |
| `unified_hardware_test` | Full system functionality | Complete system validation |
| `validate_integration` | Codebase structure validation | After code changes/deployment |
| `deploy_to_pi.sh` | Raspberry Pi deployment | Moving from dev to Pi |

## 🚀 **Quick Start Commands**

### **For Development (Linux Host)**
```bash
# Test with mock hardware
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=mock
ros2 run chessmate_hardware unified_hardware_test quick
```

### **For Real Hardware (Raspberry Pi)**
```bash
# 1. Deploy to Pi
./scripts/deploy_to_pi.sh your-pi-hostname

# 2. On Pi: Upload Arduino sketch and test
ros2 run chessmate_hardware test_arduino_serial

# 3. On Pi: Test full system
ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real
ros2 run chessmate_hardware test_ros2_hardware
```

## 🔧 **Troubleshooting**

### **Arduino Not Responding**
```bash
# Check port
ls /dev/ttyACM* /dev/ttyUSB*

# Test permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Manual test
echo 'i' > /dev/ttyACM0
cat /dev/ttyACM0
```

### **ROS 2 Issues**
```bash
# Check topics
ros2 topic list
ros2 topic echo /hardware_status

# Check services
ros2 service list
ros2 service call /calibrate_arm chessmate_msgs/srv/CalibrateArm "{calibration_type: 'home'}"
```

### **Build Issues**
```bash
# Clean rebuild
cd chessmate_dev_ws
rm -rf build install log
colcon build --packages-select chessmate_msgs chessmate_hardware
```

## 📊 **Expected Test Results**

### **Successful Arduino Serial Test**
```
🔤 Testing: Wake up ('i')
   📥 Response: 'i'
🔤 Testing: Home Z ('j')  
   📥 Response: 'j'
📊 Test Results: 6/6 (100.0%)
✅ Arduino character protocol working!
```

### **Successful ROS 2 Hardware Test**
```
🔌 Testing Basic ROS 2 Communication...
  ✅ ROS 2 communication working
🤖 Testing Arduino Commands...
  ✅ Arduino commands sent successfully
Success Rate: 100.0% (4/4)
🎉 ROS 2 HARDWARE: READY
```

### **Successful Comprehensive Test**
```
🚀 Starting Comprehensive Hardware Test Suite
✅ Arduino Communication: PASS
✅ Joint Commands: PASS  
✅ Emergency Stop: PASS
Overall Success Rate: 100.0% (5/5)
🎉 HARDWARE TEST SUITE: PASSED
```

## 🎉 **Success Criteria**

Your ChessMate system is ready when:
- ✅ Arduino serial test passes (80%+ success rate)
- ✅ ROS 2 hardware test passes (80%+ success rate)  
- ✅ Comprehensive test passes (80%+ success rate)
- ✅ Integration validation passes (100% success rate)

## 📝 **Next Steps After Testing**

1. **Chess Engine Integration** - Connect Stockfish chess engine
2. **Game Flow Testing** - Test complete chess game scenarios
3. **Performance Optimization** - Tune communication and movement speeds
4. **Production Deployment** - Deploy to final Raspberry Pi setup
