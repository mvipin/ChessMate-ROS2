# ChessMate Engine Package

This ROS 2 package provides chess engine integration for the ChessMate autonomous chess robot system. It interfaces with Stockfish and other UCI-compatible chess engines to provide computer opponent functionality, move validation, and position analysis.

## Overview

The package provides ROS 2 services for:
- **Move Calculation**: Computer move generation using Stockfish
- **Position Evaluation**: Chess position analysis and scoring
- **Move Validation**: Legal move checking and game rule enforcement
- **Game Analysis**: Opening book, endgame tablebase integration
- **Skill Level Control**: Adjustable computer opponent strength

## Package Structure

```
chessmate_engine/
├── chessmate_engine/
│   ├── __init__.py
│   ├── stockfish_interface.py       # Stockfish UCI communication
│   ├── chess_engine_node.py         # Main ROS 2 engine node
│   ├── move_validator.py            # Move validation logic
│   ├── position_analyzer.py         # Position analysis tools
│   ├── opening_book.py              # Opening book integration
│   └── engine_config.py             # Engine configuration management
├── config/
│   ├── stockfish_config.yaml        # Stockfish configuration
│   └── engine_settings.yaml         # Engine behavior settings
├── data/
│   ├── opening_book.bin             # Opening book database
│   └── endgame_tables/              # Endgame tablebase files
├── test/
│   ├── test_stockfish_interface.py  # Unit tests
│   └── test_move_validation.py      # Validation tests
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

### ROS 2 Dependencies
- `rclpy` - ROS 2 Python client library
- `chessmate_msgs` - Custom ChessMate message types
- `std_msgs` - Standard ROS 2 messages

### Python Dependencies
- `python-chess` - Chess game logic and UCI interface
- `stockfish` - Stockfish engine Python wrapper (optional)
- `numpy` - Numerical computations for analysis

### External Dependencies
- **Stockfish** - Chess engine binary
  ```bash
  sudo apt install stockfish
  ```
- **Opening Books** - Polyglot format opening books (optional)
- **Endgame Tablebases** - Syzygy tablebase files (optional)

## Installation

1. **Install system dependencies:**
   ```bash
   sudo apt update
   sudo apt install stockfish
   pip3 install python-chess numpy
   ```

2. **Build the package:**
   ```bash
   cd chessmate_dev_ws
   colcon build --packages-select chessmate_engine
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Starting the Chess Engine Node

```bash
# Start with default Stockfish configuration
ros2 run chessmate_engine chess_engine_node

# Start with custom configuration
ros2 run chessmate_engine chess_engine_node --ros-args -p stockfish_path:=/usr/games/stockfish

# Start with specific skill level
ros2 run chessmate_engine chess_engine_node --ros-args -p default_skill_level:=10
```

### Service Interfaces

#### Calculate Move Service
Request the best move for a given position:

```bash
# Example service call
ros2 service call /engine/calculate_move chessmate_msgs/srv/CalculateMove \
  "{board_fen: 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1', 
    time_limit: 5.0, 
    skill_level: 10}"
```

#### Validate Move Service
Check if a move is legal:

```bash
ros2 service call /engine/validate_move chessmate_msgs/srv/ValidateMove \
  "{board_fen: 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',
    proposed_move: {from_square: 'e2', to_square: 'e4', piece_type: 'pawn'}}"
```

#### Position Evaluation Service
Get position evaluation and analysis:

```bash
ros2 service call /engine/evaluate_position chessmate_msgs/srv/EvaluatePosition \
  "{board_fen: 'rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1'}"
```

### Configuration Parameters

#### Chess Engine Node
- `stockfish_path`: Path to Stockfish executable (default: `/usr/games/stockfish`)
- `default_skill_level`: Default engine skill level 1-20 (default: `10`)
- `default_time_limit`: Default thinking time in seconds (default: `5.0`)
- `use_opening_book`: Enable opening book (default: `true`)
- `opening_book_path`: Path to opening book file
- `use_endgame_tables`: Enable endgame tablebases (default: `false`)
- `tablebase_path`: Path to tablebase directory
- `hash_size_mb`: Engine hash table size in MB (default: `128`)
- `threads`: Number of engine threads (default: `1`)

#### Stockfish Specific Settings
- `stockfish_depth`: Maximum search depth (default: `15`)
- `stockfish_nodes`: Maximum nodes to search (default: `0` - unlimited)
- `contempt`: Contempt factor for draws (default: `0`)
- `analysis_mode`: Enable analysis mode (default: `false`)

### Python API Usage

#### Basic Engine Interface
```python
import rclpy
from rclpy.node import Node
from chessmate_msgs.srv import CalculateMove
from chessmate_engine.stockfish_interface import StockfishInterface

class ChessEngineClient(Node):
    def __init__(self):
        super().__init__('chess_engine_client')
        self.client = self.create_client(CalculateMove, 'engine/calculate_move')
        
    async def get_best_move(self, fen_position, time_limit=5.0):
        request = CalculateMove.Request()
        request.board_fen = fen_position
        request.time_limit = time_limit
        request.skill_level = 10
        
        future = self.client.call_async(request)
        response = await future
        
        if response.success:
            return response.best_move, response.evaluation
        else:
            return None, None
```

#### Direct Stockfish Interface
```python
from chessmate_engine.stockfish_interface import StockfishInterface

# Create engine interface
engine = StockfishInterface(stockfish_path='/usr/games/stockfish')

# Set position
engine.set_position('rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1')

# Get best move
best_move = engine.get_best_move(time_limit=5.0)
evaluation = engine.get_evaluation()

print(f"Best move: {best_move}")
print(f"Evaluation: {evaluation}")
```

## Engine Configuration

### Stockfish Configuration File
Create `config/stockfish_config.yaml`:

```yaml
stockfish:
  path: "/usr/games/stockfish"
  default_settings:
    Hash: 128              # Hash table size in MB
    Threads: 1             # Number of threads
    Contempt: 0            # Contempt factor
    Skill Level: 10        # Skill level (1-20)
    UCI_LimitStrength: true
    UCI_Elo: 1500         # ELO rating when using limited strength
  
  time_management:
    default_time: 5.0      # Default thinking time
    minimum_time: 0.1      # Minimum thinking time
    maximum_time: 30.0     # Maximum thinking time
    
  analysis:
    max_depth: 20          # Maximum search depth
    max_nodes: 1000000     # Maximum nodes to search
    multipv: 1             # Number of principal variations
```

### Engine Behavior Settings
Create `config/engine_settings.yaml`:

```yaml
engine_behavior:
  opening_book:
    enabled: true
    path: "data/opening_book.bin"
    variety: 0.5           # Opening variety (0.0-1.0)
    
  endgame_tables:
    enabled: false
    path: "data/endgame_tables/"
    probe_limit: 6         # Maximum pieces for tablebase lookup
    
  skill_levels:
    beginner: 3            # Skill level 1-5
    intermediate: 10       # Skill level 6-15  
    advanced: 18           # Skill level 16-20
    
  time_controls:
    blitz: 3.0            # Fast games
    rapid: 10.0           # Medium games
    classical: 30.0       # Slow games
```

## Advanced Features

### Opening Book Integration
```python
from chessmate_engine.opening_book import OpeningBook

# Load opening book
book = OpeningBook('data/opening_book.bin')

# Get opening moves
position_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
opening_moves = book.get_moves(position_fen)

print(f"Opening moves: {opening_moves}")
```

### Position Analysis
```python
from chessmate_engine.position_analyzer import PositionAnalyzer

analyzer = PositionAnalyzer()

# Analyze position
analysis = analyzer.analyze_position(
    fen="rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1",
    depth=15
)

print(f"Evaluation: {analysis.evaluation}")
print(f"Best line: {analysis.principal_variation}")
print(f"Tactical themes: {analysis.tactical_themes}")
```

### Skill Level Adjustment
```python
from chessmate_engine.chess_engine_node import ChessEngineNode

# Create engine with adaptive skill
engine = ChessEngineNode()

# Adjust skill based on game progress
if game_phase == "opening":
    engine.set_skill_level(8)  # Slightly weaker in opening
elif game_phase == "endgame":
    engine.set_skill_level(15)  # Stronger in endgame
```

## Testing

### Unit Tests
```bash
# Run all engine tests
python3 -m pytest src/chessmate_engine/test/

# Test specific components
python3 -m pytest src/chessmate_engine/test/test_stockfish_interface.py
python3 -m pytest src/chessmate_engine/test/test_move_validation.py
```

### Integration Tests
```bash
# Test engine node
ros2 run chessmate_engine test_engine_integration

# Test with real games
python3 src/chessmate_engine/test/test_complete_games.py
```

### Performance Benchmarks
```bash
# Benchmark engine performance
python3 src/chessmate_engine/test/benchmark_engine.py

# Test different skill levels
python3 src/chessmate_engine/test/test_skill_levels.py
```

## Troubleshooting

### Common Issues

1. **Stockfish Not Found**
   ```bash
   # Install Stockfish
   sudo apt install stockfish
   
   # Or specify custom path
   ros2 run chessmate_engine chess_engine_node --ros-args -p stockfish_path:=/path/to/stockfish
   ```

2. **UCI Communication Errors**
   ```bash
   # Check Stockfish version
   stockfish --help
   
   # Test UCI interface manually
   echo "uci" | stockfish
   ```

3. **Performance Issues**
   ```bash
   # Reduce hash size for low-memory systems
   ros2 run chessmate_engine chess_engine_node --ros-args -p hash_size_mb:=64
   
   # Limit search depth
   ros2 run chessmate_engine chess_engine_node --ros-args -p max_depth:=10
   ```

4. **Opening Book Errors**
   ```bash
   # Disable opening book if file missing
   ros2 run chessmate_engine chess_engine_node --ros-args -p use_opening_book:=false
   ```

### Debug Mode
```bash
# Enable detailed logging
ros2 run chessmate_engine chess_engine_node --ros-args --log-level DEBUG

# Monitor UCI communication
export STOCKFISH_DEBUG=1
ros2 run chessmate_engine chess_engine_node
```

## Performance Optimization

### Memory Usage
- Adjust hash table size based on available RAM
- Use appropriate tablebase cache sizes
- Monitor memory usage during long games

### CPU Usage
- Configure thread count based on CPU cores
- Balance search depth vs. time limits
- Use skill level to control computational load

### Response Time
- Set appropriate time limits for different game phases
- Use opening book to speed up early game
- Implement time management strategies

## Contributing

When contributing to the engine package:

1. Follow UCI protocol standards for engine communication
2. Maintain compatibility with multiple chess engines
3. Add comprehensive tests for new features
4. Document configuration parameters
5. Consider performance implications of changes
6. Test with various skill levels and time controls

## License

This package is part of the ChessMate project and follows the same licensing terms.
