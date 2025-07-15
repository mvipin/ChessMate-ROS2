# ChessMate ROS2 Architecture and Data Flow Documentation

## Executive Summary

This document provides a comprehensive overview of the ChessMate autonomous chess-playing robot system. ChessMate is a ROS2-based system that coordinates a chess engine (Stockfish), robotic arm, chessboard sensors, and game management to play complete chess games against human opponents.

## System Overview

ChessMate consists of:
- **Raspberry Pi Host**: Runs ROS2 Humble with three main production nodes
- **ChessBoard Controller (Pico)**: Connected to /dev/ttyACM0, detects human moves via hall effect sensors
- **Robot Arm Controller (Pico)**: Connected to /dev/ttyACM1, controls servo-based robotic arm
- **Stockfish Chess Engine**: Provides move calculations and game analysis

## Current System Status: ✅ FULLY WORKING

The ChessMate system is **fully functional** and can play complete chess games from start to finish.

## Production Node Architecture

### **Core Production Nodes (3 nodes):**

#### **1. topic_chess_engine_server** (chessmate_engine package)
- **Purpose**: Stockfish chess engine integration with topic-based communication
- **Topics**:
  - Subscribes: `engine/calculate_move_request`
  - Publishes: `engine/calculate_move_response`
- **Functionality**: Move calculation, position evaluation, skill level management
- **Launch**: `ros2 run chessmate_engine topic_chess_engine_server`

#### **2. topic_arduino_communication** (chessmate_hardware package)
- **Purpose**: Serial communication interface with both Pico controllers
- **Topics**:
  - Publishes: `game/human_move` (detected moves from chessboard)
  - Subscribes: `robot/execute_move_request`, `chessboard/set_mode_request`
  - Publishes: `robot/execute_move_response`, `chessboard/set_mode_response`
- **Serial Ports**: /dev/ttyACM0 (chessboard), /dev/ttyACM1 (robot)
- **Launch**: `ros2 run chessmate_hardware topic_arduino_communication`

#### **3. topic_game_management** (chessmate_engine package)
- **Purpose**: Game orchestration and complete chess game flow management
- **Topics**:
  - Publishes: `game/state` (current game status)
  - Subscribes: `game/control`, `game/human_move`
  - Uses topic-based clients for engine and hardware communication
- **Functionality**: Turn management, move validation, game state tracking
- **Launch**: `ros2 run chessmate_engine topic_game_management`

## Package Organization

### **chessmate_engine Package:**
**Production Nodes:**
- `topic_chess_engine_server` - Topic-based Stockfish integration
- `topic_game_management` - Game orchestration and flow management

**Legacy Nodes (for compatibility):**
- `chess_engine_server` - Service-based engine (not used in production)
- `chess_game_manager` - Alternative game manager
- `simple_chess_engine` - Basic engine interface

### **chessmate_hardware Package:**
**Production Nodes:**
- `topic_arduino_communication` - Serial communication with Pico controllers
- `unified_arduino_bridge` - Alternative hardware interface
- `arduino_communication_node` - Legacy hardware interface

**UI Components (for Raspberry Pi deployment):**
- `rotary_encoder_node` - Rotary encoder interface
- `lcd_display_node` - LCD display management
- `robot_animation_controller` - Robot animation sequences

### **chessmate_msgs Package:**
- Custom ROS2 message and service definitions
- `ChessMove`, `GameState`, `BoardState` messages
- `CalculateMove`, `ExecuteMove`, `SetBoardMode` services

## Topic-Based Communication Architecture

### **Why Topics Instead of Services:**
- **ROS2 Service Issues**: Service communication fails in Raspberry Pi environment
- **Reliability**: Topics provide reliable message delivery
- **Debugging**: Easier to monitor with `ros2 topic echo`
- **Flexibility**: Multiple subscribers/publishers supported

### **Active Topics:**
```
/game/state                      (GameState) - Current game status
/game/control                    (String)    - Game control commands
/game/human_move                 (String)    - Human moves from chessboard
/engine/calculate_move_request   (String)    - Engine calculation requests
/engine/calculate_move_response  (String)    - Engine calculation responses
/robot/execute_move_request      (String)    - Robot movement commands
/robot/execute_move_response     (String)    - Robot movement confirmations
/chessboard/set_mode_request     (String)    - Chessboard mode commands
/chessboard/set_mode_response    (String)    - Chessboard mode confirmations
```

## Complete End-to-End Game Flow

### **1. System Startup:**
```bash
# Default (real mode)
./launch_production_game.sh

# Mock mode (no hardware required)
./launch_production_game.sh --mode mock

# Real mode (explicit)
./launch_production_game.sh --mode real
```

**Startup Sequence:**
1. Launch three production nodes
2. Initialize serial connections to Pico controllers (/dev/ttyACM0, /dev/ttyACM1)
3. Set controller modes via serial: `"mode:real\n"` or `"mode:mock\n"`
4. Initialize chessboard for game mode
5. Start new chess game (human plays white, goes first)

### **2. Legal Moves Implementation:**
**Both Mock and Real Modes:**
- Host generates legal moves from current position using Stockfish
- Sends legal moves to chessboard controller: `"legal:e2e4,d2d4,g1f3\n"`

**Mock Mode Usage:**
- ChessBoard controller randomly selects from legal moves list
- Returns selected move: `"e2e4\n"`

**Real Mode Usage:**
- Legal moves used for LED feedback to guide human player
- Human physically makes move, detected by hall sensors
- ChessBoard sends detected move: `"e2e4\n"`

### **3. Human Move Cycle:**
```
1. topic_game_management generates legal moves
2. Sends legal moves to chessboard via topic_arduino_communication
3. ChessBoard processes legal moves (selection or LED feedback)
4. ChessBoard sends move via serial: "e2e4\n"
5. topic_arduino_communication publishes to /game/human_move
6. topic_game_management validates and processes move
7. Updates chess board state and triggers computer move
```

### **4. Computer Move Cycle:**
```
1. topic_game_management requests move via /engine/calculate_move_request
2. topic_chess_engine_server calculates best move using Stockfish
3. Engine responds via /engine/calculate_move_response with move and evaluation
4. topic_game_management sends move to robot via /robot/execute_move_request
5. Robot controller receives 6-character move: "e2pe4p"
6. Robot executes move (real hardware or mock simulation)
7. Robot confirms via /robot/execute_move_response
8. Game state updated, switch to human turn
```

## Serial Communication Protocol

### **Connection Details:**
- **Baud Rate**: 9600
- **ChessBoard Controller**: /dev/ttyACM0
- **Robot Controller**: /dev/ttyACM1
- **Format**: ASCII text, newline terminated

### **ChessBoard Commands:**
```
Host → ChessBoard:
"mode:real\n"                 → "MODE: Real hardware mode enabled\n"
"mode:mock\n"                 → "MODE: Mock simulation mode enabled\n"
"legal:e2e4,d2d4,g1f3\n"      → "LEGAL: Set 3 legal moves\n"

ChessBoard → Host:
"e2e4\n"                      (4-character UCI format move)
```

### **Robot Commands:**
```
Host → Robot:
"mode:real\n"                 → "MODE: Real hardware mode enabled\n"
"mode:mock\n"                 → "MODE: Mock simulation mode enabled\n"
"move:e2pe4p\n"               → "ACK: Executing move\n" → "DONE: Move complete\n"

6-Character Move Format: [from_square][piece][to_square][dest_piece]
- e2pe4p: pawn e2 to e4
- g1nf3n: knight g1 to f3
- e1kg1k: king e1 to g1 (castling)
- d7pq8q: pawn d7 to q8 with queen promotion
```

## System Operation and Monitoring

### **Real-time Monitoring:**
```bash
# Monitor game state
ros2 topic echo /game/state

# Watch human moves
ros2 topic echo /game/human_move

# Monitor engine calculations
ros2 topic echo /engine/calculate_move_request
ros2 topic echo /engine/calculate_move_response

# Watch robot execution
ros2 topic echo /robot/execute_move_request
ros2 topic echo /robot/execute_move_response

# View system topology
rqt_graph
```

### **Build and Deploy:**
```bash
# Build the system
./scripts/build_arm.sh chessmate_msgs chessmate_hardware chessmate_engine

# Source the workspace
source install_arm/setup.bash

# Launch production system
./launch_production_game.sh --mode mock    # For testing
./launch_production_game.sh --mode real    # For hardware
```

## Current Implementation Status

### ✅ **Completed Features:**
1. **Complete Game Flow**: Full chess games from start to finish
2. **Stockfish Integration**: Topic-based engine communication working perfectly
3. **Mock/Real Mode Coordination**: Unified mode management across controllers
4. **Legal Move Distribution**: Sent to controllers in both modes for different purposes
5. **Serial Communication**: Stable 9600 baud communication with both Pico controllers
6. **Game State Management**: Complete board state tracking and publishing
7. **Move Validation**: All moves validated against chess rules
8. **6-Character Move Format**: Proper robot controller move format

### 🔧 **Key Design Decisions:**
1. **Topic-Based Architecture**: Replaced ROS2 services due to Pi environment issues
2. **Package Reorganization**: Moved topic_game_management to chessmate_engine package
3. **Unified Mode Management**: Host coordinates mock/real mode across all controllers
4. **Direct Serial Communication**: Reliable communication without complex bridges

### 🎯 **System Capabilities:**
- **Hardware Flexibility**: Works with or without physical sensors/actuators
- **Real-time Monitoring**: All game state and moves visible via ROS2 topics
- **Robust Error Handling**: Graceful handling of hardware failures
- **Complete Chess Rules**: Full legal move validation and game end detection

This document serves as the definitive reference for understanding and working with the current ChessMate system architecture.
