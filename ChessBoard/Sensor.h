#ifndef SENSOR_H
#define SENSOR_H

#include "Utils.h"

// Sensor function prototypes
void sensor_init();
void scan_sensors();
void reset_occupancy();
bool validate_occupancy();
bool compute_move(char move[]);

// External variables declared in Sensor.ino
extern bool occupancy_init[CHESS_ROWS][CHESS_COLS];
extern uint8_t occupancy_delta[CHESS_ROWS][CHESS_COLS];

#endif // SENSOR_H
