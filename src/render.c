#include "render.h"

void render_stars(star_t const stars[], size_t n) {
    _render_clear();

    for (size_t i = 0; i < n; i++) {
        star_t const* star = &stars[i];

        // determine color
        render_color_t color = RENDER_COLOR_WHITE;
        if (star->intensity < 0.33) {
            color = RENDER_COLOR_DARK_GRAY;
        } else if (star->intensity < 0.66) {
            color = RENDER_COLOR_LIGHT_GRAY;
        }

        // convert normalized coords to screen coords
        uint16_t screen_x = (uint16_t)((star->x + 1.0) / 2.0 * (SCREEN_WIDTH - 1));
        uint16_t screen_y = (uint16_t)((-star->y + 1.0) / 2.0 * (SCREEN_HEIGHT - 1));

        // compute reference resolution to renderer resolution scaling factor
        double scale = (SCREEN_WIDTH < SCREEN_HEIGHT ? SCREEN_WIDTH : SCREEN_HEIGHT) / 640.0;

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
                    _render_pixel(px, py, color);
                }
                py = (int32_t)screen_y - y;
                if (py >= 0 && py < SCREEN_HEIGHT) {
                    _render_pixel(px, py, color);
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
                    _render_pixel(px, py, color);
                }
                py = (int32_t)screen_y - x;
                if (py >= 0 && py < SCREEN_HEIGHT) {
                    _render_pixel(px, py, color);
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

    _render_commit();
}
