#pragma once

#include "bsc.h"

#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#define COORD_BUFFER_SIZE BSC_SIZE

typedef struct {
    double x;
    double y;
    double intensity;
} star_t;

bool get_stars(
  double latitude,
  double longitude,
  struct tm const* time,
  double subsecond,
  double vmag_max,
  star_t out[COORD_BUFFER_SIZE],
  size_t* out_size);
