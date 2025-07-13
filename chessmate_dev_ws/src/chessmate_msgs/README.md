# ChessMate Messages Package

This ROS 2 package defines custom message types, service definitions, and action interfaces for the ChessMate autonomous chess robot system. It provides standardized communication interfaces between all ChessMate components.

## Overview

The package provides ROS 2 interfaces for:
- **Chess Game Messages**: Board state, moves, and game status
- **Hardware Interface Messages**: Controller commands and status
- **User Interface Messages**: Display updates and input events
- **Robot Control Services**: Move execution and calibration
- **Chess Engine Services**: Move calculation and position evaluation

## Package Structure

```
chessmate_msgs/
├── msg/
│   ├── BoardState.msg              # Chess board state representation
│   ├── ChessMove.msg               # Chess move definition
│   ├── GameState.msg               # Complete game state
│   ├── RobotStatus.msg             # Robot controller status
│   ├── LCDCommand.msg              # LCD display commands
│   ├── RotaryEncoderEvent.msg      # Rotary encoder input events
│   └── ArduinoCommand.msg          # Arduino controller commands
├── srv/
│   ├── ExecuteMove.srv             # Robot move execution service
│   ├── CalculateMove.srv           # Chess engine move calculation
│   ├── ValidateMove.srv            # Move validation service
│   ├── SetBoardMode.srv            # Board controller mode setting
│   └── CalibrateArm.srv            # Robot arm calibration service
├── action/
│   └── ChessGame.action            # Complete chess game action
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Message Definitions

### Chess Game Messages

#### BoardState.msg
Represents the current state of the chess board:
```
# Board representation using FEN notation
string fen_position

# Individual square occupancy (64 squares, a1-h8)
uint8[64] square_occupancy

# Piece positions for each color
string[] white_pieces
string[] black_pieces

# Game metadata
uint32 move_number
bool white_to_move
string castling_rights
string en_passant_target
uint32 halfmove_clock
uint32 fullmove_number

# Timestamp
builtin_interfaces/Time timestamp
```

#### ChessMove.msg
Defines a chess move with complete information:
```
# Move notation
string from_square      # e.g., "e2"
string to_square        # e.g., "e4"
string piece_type       # "pawn", "rook", "knight", "bishop", "queen", "king"
string move_type        # "normal", "capture", "castling", "en_passant", "promotion"

# Move flags
bool is_capture
bool is_check
bool is_checkmate
bool is_promotion
string promotion_piece  # If promotion, what piece to promote to

# Move validation
bool is_legal
string algebraic_notation  # Standard chess notation (e.g., "Nf3", "O-O")
string uci_notation       # UCI format (e.g., "e2e4")

# Timing information
builtin_interfaces/Time timestamp
float64 move_time        # Time taken to make the move
```

#### GameState.msg
Complete game state information:
```
# Current board state
BoardState board_state

# Game status
string game_phase        # "opening", "middlegame", "endgame"
string game_status       # "active", "check", "checkmate", "stalemate", "draw"
string result            # "ongoing", "white_wins", "black_wins", "draw"

# Player information
string white_player      # "human", "computer", "remote"
string black_player      # "human", "computer", "remote"
uint8 computer_skill     # 1-20 skill level for computer player

# Move history
ChessMove[] move_history
string[] legal_moves     # Current legal moves in algebraic notation

# Game timing
builtin_interfaces/Time game_start_time
float64 white_time_remaining
float64 black_time_remaining
```

### Hardware Interface Messages

#### RobotStatus.msg
Robot controller status and feedback:
```
# Robot state
string status           # "idle", "moving", "homing", "error", "calibrating"
string last_command     # Last executed command
bool is_ready          # Ready for new commands

# Position information
float64 x_position     # Current X coordinate
float64 y_position     # Current Y coordinate
float64 z_position     # Current Z coordinate
bool gripper_closed    # Gripper state

# Error information
bool has_error
string error_message
uint32 error_code

# Performance metrics
float64 move_duration  # Last move execution time
uint32 moves_completed # Total moves completed
builtin_interfaces/Time last_update
```

#### LCDCommand.msg
LCD display control commands:
```
# Display content
string line1           # First line of text
string line2           # Second line of text
string line3           # Third line of text (if supported)
string line4           # Fourth line of text (if supported)

# Display control
bool clear_display     # Clear before writing
bool backlight_on      # Backlight control
uint8 cursor_position  # Cursor position (0-based)
bool cursor_visible    # Show cursor

# Special commands
string command_type    # "text", "menu", "status", "game"
string[] menu_items    # Menu items for menu display
uint8 selected_item    # Currently selected menu item
```

#### RotaryEncoderEvent.msg
Rotary encoder input events:
```
# Rotation information
int32 rotation_delta   # Rotation change (-1, 0, +1)
int32 total_rotation   # Total rotation since start
bool clockwise         # Direction of rotation

# Button information
bool button_pressed    # Button press event
bool button_released   # Button release event
bool button_held       # Button currently held
float64 hold_duration  # How long button has been held

# Event timing
builtin_interfaces/Time timestamp
uint32 sequence_number # Event sequence for ordering
```

## Service Definitions

### ExecuteMove.srv
Robot move execution service:
```
# Request
ChessMove move
bool validate_move     # Validate before executing
float64 timeout        # Maximum execution time

---
# Response
bool success
string message
float64 execution_time
RobotStatus final_status
```

### CalculateMove.srv
Chess engine move calculation:
```
# Request
string board_fen       # Current board position
float64 time_limit     # Maximum thinking time
uint8 skill_level      # Engine skill level (1-20)
string[] legal_moves   # Available legal moves

---
# Response
bool success
string best_move       # Best move in algebraic notation
string uci_move        # Best move in UCI format
float64 evaluation     # Position evaluation
string principal_variation  # Main line of play
float64 calculation_time
```

### ValidateMove.srv
Move validation service:
```
# Request
string board_fen       # Current board position
ChessMove proposed_move

---
# Response
bool is_legal
string validation_message
string[] legal_moves   # All legal moves from position
```

## Action Definitions

### ChessGame.action
Complete chess game management:
```
# Goal
string white_player    # "human" or "computer"
string black_player    # "human" or "computer"
uint8 skill_level      # Computer skill level
float64 time_control   # Time per move

---
# Result
string game_result     # "white_wins", "black_wins", "draw"
ChessMove[] move_history
float64 game_duration
string termination_reason

---
# Feedback
GameState current_state
string current_player
float64 time_remaining
uint32 moves_played
```

## Dependencies

### ROS 2 Dependencies
- `std_msgs` - Standard ROS 2 message types
- `builtin_interfaces` - Built-in ROS 2 interfaces (Time, Duration)
- `geometry_msgs` - Geometry-related messages (for robot positioning)

### Build Dependencies
- `rosidl_default_generators` - Message generation tools
- `rosidl_default_runtime` - Runtime message support

## Installation

1. **Build the package:**
   ```bash
   cd chessmate_dev_ws
   colcon build --packages-select chessmate_msgs
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### In Python Nodes
```python
from chessmate_msgs.msg import BoardState, ChessMove, GameState
from chessmate_msgs.srv import ExecuteMove, CalculateMove

# Create a chess move message
move = ChessMove()
move.from_square = "e2"
move.to_square = "e4"
move.piece_type = "pawn"
move.move_type = "normal"
move.is_legal = True
move.algebraic_notation = "e4"
```

### In C++ Nodes
```cpp
#include "chessmate_msgs/msg/board_state.hpp"
#include "chessmate_msgs/srv/execute_move.hpp"

// Create a board state message
auto board_state = chessmate_msgs::msg::BoardState();
board_state.fen_position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
```

### Message Inspection
```bash
# View message structure
ros2 interface show chessmate_msgs/msg/BoardState
ros2 interface show chessmate_msgs/srv/ExecuteMove

# List all ChessMate messages
ros2 interface list | grep chessmate_msgs

# Monitor live messages
ros2 topic echo /board_state
ros2 topic echo /chess_moves
```

## Message Flow Architecture

The ChessMate system uses these messages in the following flow:

```
User Input → RotaryEncoderEvent → Game Logic
                                      ↓
Display ← LCDCommand ← Game State Management
                                      ↓
Chess Engine ← CalculateMove ← Game Logic → ExecuteMove → Robot
                                      ↓
Board Sensing ← ArduinoCommand ← Game Logic
```

## Integration Examples

### Publishing a Chess Move
```python
import rclpy
from rclpy.node import Node
from chessmate_msgs.msg import ChessMove

class MovePublisher(Node):
    def __init__(self):
        super().__init__('move_publisher')
        self.publisher = self.create_publisher(ChessMove, 'chess_moves', 10)
        
    def publish_move(self, from_sq, to_sq):
        move = ChessMove()
        move.from_square = from_sq
        move.to_square = to_sq
        move.timestamp = self.get_clock().now().to_msg()
        self.publisher.publish(move)
```

### Calling Move Execution Service
```python
from chessmate_msgs.srv import ExecuteMove

class MoveExecutor(Node):
    def __init__(self):
        super().__init__('move_executor')
        self.client = self.create_client(ExecuteMove, 'execute_move')
        
    async def execute_move(self, move):
        request = ExecuteMove.Request()
        request.move = move
        request.validate_move = True
        
        future = self.client.call_async(request)
        response = await future
        return response.success
```

## Testing

### Message Validation
```bash
# Test message publishing
ros2 topic pub /chess_moves chessmate_msgs/msg/ChessMove \
  "{from_square: 'e2', to_square: 'e4', piece_type: 'pawn'}"

# Test service calls
ros2 service call /validate_move chessmate_msgs/srv/ValidateMove \
  "{board_fen: 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'}"
```

## Contributing

When adding new message types:

1. Follow ROS 2 naming conventions (PascalCase for messages, snake_case for fields)
2. Include comprehensive documentation in message files
3. Add timestamp fields for time-sensitive messages
4. Use appropriate data types (prefer specific types over generic ones)
5. Include validation and error handling fields where appropriate
6. Update this README with new message descriptions

## License

This package is part of the ChessMate project and follows the same licensing terms.
