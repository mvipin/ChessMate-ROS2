# ChessMate ChessBoard Arduino Controller

This Arduino sketch controls the chessboard hardware for the ChessMate chess robot system. It handles board sensing, LED matrix display, and user interface components.

## Features

- **Board Sensing**: Hall effect sensors detect piece positions
- **LED Matrix Display**: 8x8 RGB LED matrix for move highlighting and game status
- **User Interface**: Status LEDs and button controls
- **Serial Communication**: Communicates with Raspberry Pi via Serial2
- **Game State Management**: Tracks move states and validates player moves

## Hardware Requirements

- Arduino-compatible microcontroller
- 8x8 RGB LED matrix (WS2812B/NeoPixel)
- Hall effect sensors for each square (64 total)
- Status LEDs for human/computer turn indicators
- Buttons for move confirmation and hints
- Serial connection to Raspberry Pi

## Communication Protocol

### Commands from Raspberry Pi (via Serial2):

- `init` - Initialize board for new game
- `occupancy:pos1:pos2:...` - Set initial piece positions
- `legal:move1:move2:...` - Set legal moves for current position
- `hint:move` - Set hint move to display
- `check:square1:square2:...` - Highlight check squares
- `start` - Begin human turn
- `comp:move` - Execute computer move
- `override:move` - Override/undo move
- `checkmate:king_pos:move` - Display checkmate
- `reset` - Reset board state

### Responses to Raspberry Pi:

- `move` - 4-character move notation (e.g., "e2e4")
- `ffff` - Hint override signal
- Board state updates and confirmations

## Architecture Changes

**Note**: This version has been updated to remove direct robotic arm communication. The ChessBoard Arduino now focuses solely on:

1. **Board sensing and validation**
2. **LED display and user interface**
3. **Communication with Raspberry Pi**

The Raspberry Pi now handles all robotic arm coordination, making the system more modular and reliable.

## Pin Configuration

- **Serial2**: Communication with Raspberry Pi (pins 8, 9)
- **LED Matrix**: Data pin for WS2812B strip
- **Sensors**: Hall effect sensors on analog/digital pins
- **Status LEDs**: Human/computer turn indicators
- **Buttons**: Confirm and hint buttons

## Installation

1. Install required libraries:
   - FastLED (for LED matrix)
   - Any sensor-specific libraries

2. Configure pin assignments in the code
3. Upload to Arduino
4. Connect to ChessMate Raspberry Pi system

## Integration

This ChessBoard controller integrates with the ChessMate ROS 2 system running on Raspberry Pi. The Pi coordinates between:

- ChessBoard Arduino (this controller)
- Robotic Arm Arduino (separate controller)
- Chess engine (Stockfish)
- User interface (LCD, rotary encoder)

This separation of concerns improves system reliability and makes debugging easier.
