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
  struct tm const* time,
  double subsecond,
  double latitude,
  double longitude,
  star_callback_t on_star);
