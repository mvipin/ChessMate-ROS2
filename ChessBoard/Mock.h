#ifndef MOCK_H
#define MOCK_H

#include "Utils.h"

// Mock mode function prototypes
void mock_init_occupancy();
void mock_update_occupancy(const char* move);
char* mock_select_random_move();
void mock_start_move();
bool mock_check_move_complete();

#endif // MOCK_H
