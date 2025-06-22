#include "render.h"

#include "board.h"

#include <math.h>

void render_star(star_t const* star) {
#ifndef NO_CROP
    float const aspect_ratio = (float)RENDER_WIDTH / RENDER_HEIGHT;
    float const b = 1.0f / sqrtf(aspect_ratio * aspect_ratio + 1.0f);
    float const a = aspect_ratio * b;
    float const screen_scale = fminf((RENDER_WIDTH - 1) / a, (RENDER_HEIGHT - 1) / b);
#else  // NO_CROP
    float const screen_scale = fminf(RENDER_WIDTH - 1, RENDER_HEIGHT - 1);
#endif // NO_CROP

#ifndef NO_CROP
    // skip stars outside the inscribed rectangle
    if (fabsf(star->x) > a || fabsf(star->y) > b) {
        return;
    }
#endif // NO_CROP

    // convert normalized coords to screen coords
    uint16_t screen_x = (uint16_t)(((RENDER_WIDTH - 1) + star->x * screen_scale) / 2);
    uint16_t screen_y = (uint16_t)(((RENDER_HEIGHT - 1) + star->y * screen_scale) / 2);

    // compute reference resolution to renderer resolution scaling factor
    float scale = (RENDER_WIDTH < RENDER_HEIGHT ? RENDER_WIDTH : RENDER_HEIGHT) / 640.0f;

    // determine color
    render_color_t color = RENDER_COLOR_WHITE;
    if (star->intensity < 0.33f) {
        color = RENDER_COLOR_DARK_GRAY;
    } else if (star->intensity < 0.66f) {
        color = RENDER_COLOR_LIGHT_GRAY;
    }

    // draw circle
    int8_t radius = 1 + (int8_t)(scale * (star->intensity * 3.5f));
    int8_t x = 0;
    int8_t y = radius;
    int8_t d = 1 - radius;
    int32_t px, py;

    while (x <= y) {
        for (int8_t dx = -x; dx <= x; dx++) {
            px = (int32_t)screen_x + dx;
            if (px < 0 || px >= RENDER_WIDTH) {
                // out of bounds
                continue;
            }
            py = (int32_t)screen_y + y;
            if (py >= 0 && py < RENDER_HEIGHT) {
                board_render_pixel(px, py, color);
            }
            py = (int32_t)screen_y - y;
            if (py >= 0 && py < RENDER_HEIGHT) {
                board_render_pixel(px, py, color);
            }
        }
        for (int8_t dx = -y; dx <= y; dx++) {
            px = (int32_t)screen_x + dx;
            if (px < 0 || px >= RENDER_WIDTH) {
                // out of bounds
                continue;
            }
            py = (int32_t)screen_y + x;
            if (py >= 0 && py < RENDER_HEIGHT) {
                board_render_pixel(px, py, color);
            }
            py = (int32_t)screen_y - x;
            if (py >= 0 && py < RENDER_HEIGHT) {
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
