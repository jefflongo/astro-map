#include "board_render.h"

uint16_t const SCREEN_WIDTH = 648;
uint16_t const SCREEN_HEIGHT = 640;
uint32_t const RENDER_FREQ_MS = 600000;

bool board_render_init(void) {
    return true;
}

void board_render_deinit(void) {}

bool board_render_should_run(void) {
    return true;
}

void board_render_clear(void) {}

void board_render_pixel(uint16_t x, uint16_t y, render_color_t color) {
    (void)x;
    (void)y;
    (void)color;
}

void board_render_commit(void) {}
