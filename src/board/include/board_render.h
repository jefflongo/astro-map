#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RENDER_COLOR_BLACK,
    RENDER_COLOR_DARK_GRAY,
    RENDER_COLOR_LIGHT_GRAY,
    RENDER_COLOR_WHITE,
} render_color_t;

extern uint16_t const RENDER_WIDTH;
extern uint16_t const RENDER_HEIGHT;
extern uint32_t const RENDER_FREQ_MS;

bool board_render_init(void);
void board_render_deinit(void);
bool board_render_should_run(void);

void board_render_pixel(uint16_t x, uint16_t y, render_color_t color);
void board_render_clear(void);
void board_render_commit(void);
