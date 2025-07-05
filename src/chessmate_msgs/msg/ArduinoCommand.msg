# Arduino communication command message
# Timestamp of the command
builtin_interfaces/Time timestamp

# Command types for Arduino communication
uint8 CMD_INIT = 0
uint8 CMD_OCCUPANCY = 1
uint8 CMD_LEGAL_MOVES = 2
uint8 CMD_HINT = 3
uint8 CMD_CHECK = 4
uint8 CMD_START_MOVE = 5
uint8 CMD_COMPUTER_MOVE = 6
uint8 CMD_OVERRIDE_MOVE = 7
uint8 CMD_RESET = 8
uint8 CMD_CHECKMATE = 9
uint8 CMD_HEARTBEAT = 10

uint8 command_type

# Command payload data
string data

# Target Arduino (0=board controller, 1=arm controller)
uint8 target_arduino
