#include "render.h"

#include "board.h"

#include <math.h>

void render_star(star_t const* star) {
#ifndef NO_CROP
    double const aspect_ratio = (double)SCREEN_WIDTH / SCREEN_HEIGHT;
    double const b = 1.0 / sqrt(aspect_ratio * aspect_ratio + 1.0);
    double const a = aspect_ratio * b;
    double const screen_scale = fmin((SCREEN_WIDTH - 1) / a, (SCREEN_HEIGHT - 1) / b);
#else  // NO_CROP
    double const screen_scale = fmin(SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
#endif // NO_CROP

#ifndef NO_CROP
    // skip stars outside the inscribed rectangle
    if (fabs(star->x) > a || fabs(star->y) > b) {
        return;
    }
#endif // NO_CROP

    // convert normalized coords to screen coords
    uint16_t screen_x = (uint16_t)(((SCREEN_WIDTH - 1) + star->x * screen_scale) / 2);
    uint16_t screen_y = (uint16_t)(((SCREEN_HEIGHT - 1) - star->y * screen_scale) / 2);

    // compute reference resolution to renderer resolution scaling factor
    double scale = (SCREEN_WIDTH < SCREEN_HEIGHT ? SCREEN_WIDTH : SCREEN_HEIGHT) / 640.0;

    // determine color
    render_color_t color = RENDER_COLOR_WHITE;
    if (star->intensity < 0.33) {
        color = RENDER_COLOR_DARK_GRAY;
    } else if (star->intensity < 0.66) {
        color = RENDER_COLOR_LIGHT_GRAY;
    }

    // draw circle
    int8_t radius = 1 + (int8_t)(scale * (star->intensity * 3));
    int8_t x = 0;
    int8_t y = radius;
    int8_t d = 1 - radius;
    int32_t px, py;

    while (x <= y) {
        for (int8_t dx = -x; dx <= x; dx++) {
            px = (int32_t)screen_x + dx;
            if (px < 0 || px >= SCREEN_WIDTH) {
                // out of bounds
                continue;
            }
            py = (int32_t)screen_y + y;
            if (py >= 0 && py < SCREEN_HEIGHT) {
                board_render_pixel(px, py, color);
            }
            py = (int32_t)screen_y - y;
            if (py >= 0 && py < SCREEN_HEIGHT) {
                board_render_pixel(px, py, color);
            }
        }
        for (int8_t dx = -y; dx <= y; dx++) {
            px = (int32_t)screen_x + dx;
            if (px < 0 || px >= SCREEN_WIDTH) {
                // out of bounds
                continue;
            }
            py = (int32_t)screen_y + x;
            if (py >= 0 && py < SCREEN_HEIGHT) {
                board_render_pixel(px, py, color);
            }
            py = (int32_t)screen_y - x;
            if (py >= 0 && py < SCREEN_HEIGHT) {
                board_render_pixel(px, py, color);
            }
        }

        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}
