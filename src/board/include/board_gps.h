#pragma once

#include <stdbool.h>

bool board_gps_init(void);
void board_gps_deinit();

bool board_gps_location(double* latitude, double* longitude);
