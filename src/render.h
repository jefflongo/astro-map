#pragma once

#include "astro.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern uint16_t const SCREEN_WIDTH;
extern uint16_t const SCREEN_HEIGHT;

typedef enum {
    RENDER_COLOR_BLACK = 0x00,
    RENDER_COLOR_DARK_GRAY = 0x55,
    RENDER_COLOR_LIGHT_GRAY = 0xAA,
    RENDER_COLOR_WHITE = 0xFF,
} render_color_t;

bool render_init(void);
void render_deinit(void);

bool render_loop(void);
void render_stars(star_t const stars[], size_t n);

void _render_clear(void);
void _render_pixel(uint16_t x, uint16_t y, render_color_t color);
void _render_commit(void);
