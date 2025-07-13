# ChessMate: Autonomous Chess Robot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Hackaday](https://img.shields.io/badge/Hackaday-Project-orange)](https://hackaday.io/project/your-project-id)

An autonomous chess-playing robot system built with ROS2, featuring a 6-DOF robotic arm, computer vision board sensing, and distributed computing architecture. ChessMate can play chess against human opponents with full game management, move validation, and real-time visualization.

## 🏗️ **System Architecture**

ChessMate uses a distributed architecture with multiple specialized components:

### **Hardware Architecture**
- **🤖 6-DOF Robotic Arm**: SCARA-style arm for precise piece manipulation
- **📋 Smart Chessboard**: Magnetic reed sensor array for piece detection
- **🧠 Raspberry Pi 4**: Main computing unit running ROS2 nodes
- **⚡ Dual Pi Pico Controllers**: Real-time control for board sensing and arm movement
- **🖥️ Development Host**: Optional GUI and visualization support

### **Software Architecture**
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Development   │    │  Raspberry Pi   │    │   Controllers   │
│      Host       │    │      Host       │    │   (Pi Picos)    │
├─────────────────┤    ├─────────────────┤    ├─────────────────┤
│ • RViz2         │◄──►│ • Game Manager  │◄──►│ • ChessBoard    │
│ • rqt Dashboard │    │ • Chess Engine  │    │ • Robot Arm     │
│ • Monitoring    │    │ • Hardware I/F  │    │ • Mock/Real     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🚀 **Key Features**

### **🎮 Game Management**
- **Stockfish Integration**: UCI-compatible chess engine with adjustable skill levels
- **Move Validation**: Real-time legal move checking and game rule enforcement
- **Game State Tracking**: Complete FEN notation and move history management
- **Multiple Game Modes**: Human vs Computer, analysis mode, position setup

### **🤖 Robot Control**
- **Precise Manipulation**: Sub-millimeter repeatability for piece placement
- **Collision Avoidance**: Safe trajectory planning and execution
- **Adaptive Gripping**: Automatic piece detection and secure handling
- **Home Position**: Automatic calibration and error recovery

### **📡 Distributed Computing**
- **ROS2 Communication**: Reliable inter-node messaging and service calls
- **Mock/Real/Sim Modes**: Development without hardware dependencies
- **Scalable Architecture**: Easy addition of new sensors and actuators
- **Real-time Performance**: Low-latency control loops for responsive gameplay

### **🔧 Development Features**
- **Hardware Abstraction**: Same code runs on Pi, development host, and simulation
- **Comprehensive Testing**: Unit tests, integration tests, and mock hardware
- **Modular Design**: Independent packages for easy maintenance and extension
- **Rich Visualization**: Real-time 3D robot model and game state display

## 📦 **ROS2 Packages**

| Package | Purpose | Documentation |
|---------|---------|---------------|
| [`chessmate_msgs`](chessmate_dev_ws/src/chessmate_msgs/) | Message definitions and service interfaces | [README](chessmate_dev_ws/src/chessmate_msgs/README.md) |
| [`chessmate_hardware`](chessmate_dev_ws/src/chessmate_hardware/) | Hardware interfaces and device drivers | [README](chessmate_dev_ws/src/chessmate_hardware/README.md) |
| [`chessmate_engine`](chessmate_dev_ws/src/chessmate_engine/) | Chess engine integration (Stockfish) | [README](chessmate_dev_ws/src/chessmate_engine/README.md) |
| [`chessmate_description`](chessmate_dev_ws/src/chessmate_description/) | Robot models and visualization | [README](chessmate_dev_ws/src/chessmate_description/README.md) |
| [`chessmate_kinematics`](chessmate_dev_ws/src/chessmate_kinematics/) | Motion planning and kinematics | [README](chessmate_dev_ws/src/chessmate_kinematics/README.md) |

## 🛠️ **Building the Software**

### **Prerequisites**
- **ROS2 Humble** (Ubuntu 22.04 recommended)
- **Python 3.10+** with pip
- **Git** for version control
- **Raspberry Pi 4** (for hardware deployment)

### **Quick Start**
```bash
# Clone the repository
git clone https://github.com/your-username/ChessMate.git
cd ChessMate

# Run automated setup (installs dependencies and builds workspace)
cd chessmate_dev_ws
./setup_and_build.sh

# Source the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash  # (or install_arm/setup.bash on Pi)

# Test the installation
./scripts/test_controllers.sh
```

### **Development Setup**
For detailed development setup including IDE configuration, debugging tools, and advanced build options, see:
- [Development Setup Guide](docs/ROS2_CONVERSION_ROADMAP.md)
- [Testing Strategy](docs/ROS2_INTEGRATION_TESTING_STRATEGY.md)
- [Hardware Integration](docs/HARDWARE_INTEGRATION_LOG.md)

## 🔧 **Building the Hardware**

### **Mechanical Design**
Complete CAD files, 3D printing instructions, and assembly guides are available on the Hackaday project page:

**🔗 [ChessMate Hardware Documentation](https://hackaday.io/project/your-project-id)**

### **Electronics**
- **Schematics**: Available in the Hackaday project files
- **PCB Designs**: Custom boards for sensor integration
- **Wiring Diagrams**: Complete connection documentation
- **Component Lists**: BOM with supplier information

### **Assembly Guide**
*[Placeholder - Hardware assembly documentation will be added as construction progresses]*

## 🧪 **Testing**

### **System Testing**

ChessMate provides comprehensive system testing with increasing complexity levels:

```bash
# Level 0: Individual Pi Pico Controllers (no ROS2)
./scripts/test_chessmate_system.sh pico --mode mock --controller both
./scripts/test_chessmate_system.sh pico --mode real --controller chessboard

# Level 1: ROS2 Integration Testing
./scripts/test_chessmate_system.sh ros2 --mode mock --controller both
./scripts/test_chessmate_system.sh ros2 --mode real --controller robot

# Level 2: Complete Chess Game Simulation
./scripts/test_chessmate_system.sh game --mode mock --duration 300
./scripts/test_chessmate_system.sh game --mode real --duration 600 --skill-level 10
```

**Get help for any test script:**
```bash
./scripts/test_chessmate_system.sh --help
./chessmate_dev_ws/scripts/test_pi_host.sh --help
./chessmate_dev_ws/scripts/test_x86_host.sh --help
```

**Clean up processes between tests:**
```bash
./scripts/cleanup_chessmate_processes.sh          # Full cleanup
./scripts/cleanup_chessmate_processes.sh --help   # See cleanup options
```

**For detailed component testing and configuration:**
- **Pi Host Components**: See [ROS Workspace README](chessmate_dev_ws/README.md)
- **x86 Host Components**: See [ROS Workspace README](chessmate_dev_ws/README.md)
- **Hardware Setup**: See [Hardware Documentation](docs/)

### **Expected Results**
- **Controller Tests**: ✅ ChessBoard and Robot controllers respond to commands
- **Integration Tests**: ✅ ROS2 nodes communicate properly
- **Timing Tests**: ✅ Move execution within 3-15 seconds
- **Visualization**: ✅ Robot model displays correctly in RViz2

### **Test Coverage**
- **Unit Tests**: Individual component validation
- **Integration Tests**: Multi-component system validation
- **Hardware Tests**: Real device communication verification
- **Mock Tests**: Development without hardware dependencies

For detailed testing procedures, see [Testing Guide](chessmate_dev_ws/TESTING_GUIDE.md).

## 📊 **Results & Visualizations**

### **System Demonstrations**
*[Placeholder - Demo videos and animations will be added as system development progresses]*

- **🎥 YouTube Demos**: Complete game demonstrations
- **📹 RViz Animations**: Robot motion planning visualization
- **📈 Performance Metrics**: Move timing and accuracy analysis
- **🎮 Game Recordings**: Human vs ChessMate matches

### **ROS2 System Visualization**
```bash
# View node graph
rqt_graph

# Monitor system performance
rqt

# 3D robot visualization
rviz2 -d src/chessmate_description/rviz/chessmate_default.rviz
```

### **Real-time Monitoring**
- **Node Communication**: Live message flow visualization
- **Robot State**: Joint positions and end-effector tracking
- **Game Progress**: Move history and board state updates
- **System Health**: Hardware status and error monitoring

## 🏆 **Project Milestones**

### **✅ Completed**
- [x] ROS2 workspace setup and package structure
- [x] Pi Pico controller firmware (ChessBoard + Robot)
- [x] USB Serial communication protocols
- [x] Mock hardware testing framework
- [x] Basic robot kinematics and control
- [x] Chess engine integration (Stockfish)
- [x] Distributed development environment

### **🚧 In Progress**
- [ ] Physical robot construction and assembly
- [ ] Computer vision board sensing integration
- [ ] Advanced motion planning and collision avoidance
- [ ] Complete game flow automation
- [ ] Performance optimization and tuning

### **📋 Planned**
- [ ] Tournament mode with multiple difficulty levels
- [ ] Web interface for remote play
- [ ] Machine learning move prediction
- [ ] Multi-robot coordination for larger boards
- [ ] Educational mode with move explanations

## 📚 **Documentation**

### **Technical Documentation**
- [System Architecture](docs/HACKADAY_LOG_NEW_DESIGN.md)
- [ROS2 Integration](docs/ROS2_CONVERSION_ROADMAP.md)
- [Hardware Interfaces](docs/ROBOT_CONTROLLER_USB_ARCHITECTURE.md)
- [Testing Framework](docs/ROS2_INTEGRATION_TESTING_STRATEGY.md)
- [Development Logs](docs/)

### **API Documentation**
- [Message Interfaces](chessmate_dev_ws/src/chessmate_msgs/README.md)
- [Service Definitions](chessmate_dev_ws/src/chessmate_msgs/README.md#service-definitions)
- [Hardware Abstraction](chessmate_dev_ws/src/chessmate_hardware/README.md#hardware-abstraction)

## 🤝 **Contributing**

We welcome contributions! Please see our contributing guidelines:

1. **Fork the repository** and create a feature branch
2. **Follow ROS2 coding standards** and naming conventions
3. **Add tests** for new functionality
4. **Update documentation** for any API changes
5. **Submit a pull request** with a clear description

### **Development Environment**
- Use the provided development container or follow setup instructions
- Run tests before submitting: `./test.sh all`
- Ensure code passes linting and formatting checks

## 📄 **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 **Acknowledgments**

- **ROS2 Community** for the excellent robotics framework
- **Stockfish Team** for the powerful chess engine
- **Raspberry Pi Foundation** for accessible computing hardware
- **Open Source Community** for countless libraries and tools

## 📞 **Contact**

- **Project Lead**: [Your Name](mailto:your.email@example.com)
- **Hackaday Project**: [ChessMate Project Page](https://hackaday.io/project/your-project-id)
- **Issues**: [GitHub Issues](https://github.com/your-username/ChessMate/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/ChessMate/discussions)

---

**⭐ Star this repository if you find ChessMate interesting!**

*Built with ❤️ using ROS2, Python, and lots of coffee ☕*
