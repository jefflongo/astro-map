#pragma once

#include "bsc.h"

#include <time.h>

typedef struct {
    double x;
    double y;
    double intensity;
} star_t;

typedef void (*star_callback_t)(star_t const*);

void get_stars(
  double latitude,
  double longitude,
  struct tm const* time,
  double subsecond,
  star_callback_t on_star);
