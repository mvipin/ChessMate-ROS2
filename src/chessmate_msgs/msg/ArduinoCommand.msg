# Arduino communication command message
# Timestamp of the command
builtin_interfaces/Time timestamp

# Command types for Arduino communication
uint8 CMD_OCCUPANCY = 0
uint8 CMD_LEGAL_MOVES = 1
uint8 CMD_HINT = 2
uint8 CMD_CHECK = 3
uint8 CMD_START_MOVE = 4
uint8 CMD_COMPUTER_MOVE = 5
uint8 CMD_OVERRIDE_MOVE = 6
uint8 CMD_RESET = 7
uint8 CMD_CHECKMATE = 8

uint8 command_type

# Command payload data
string data

# Target Arduino (0=board controller, 1=arm controller)
uint8 target_arduino
