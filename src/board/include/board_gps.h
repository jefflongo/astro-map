#pragma once

#include <stdbool.h>
#include <time.h>

bool board_gps_init(void);
void board_gps_deinit();

bool board_gps_time_location(
  struct tm* time, double* subsecond, double* latitude, double* longitude);
