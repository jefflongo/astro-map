#pragma once

#include "bsc.h"

#include <time.h>

typedef struct {
    float x;
    float y;
    float intensity;
} star_t;

typedef void (*star_callback_t)(star_t const*);

void get_stars(
  struct tm const* time, float subsecond, float latitude, float longitude, star_callback_t on_star);
