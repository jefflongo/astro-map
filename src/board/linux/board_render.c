#include "board_render.h"

#include <SDL2/SDL.h>

uint16_t const RENDER_WIDTH = 648;
uint16_t const RENDER_HEIGHT = 480;
uint32_t const RENDER_FREQ_MS = 16;

static SDL_Renderer* renderer = NULL;
static SDL_Window* window = NULL;

bool board_render_init(void) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "ERROR: SDL failed to initialize: %s\r\n", SDL_GetError());
        return false;
    }

    window = SDL_CreateWindow(
      "Sky Map",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      RENDER_WIDTH,
      RENDER_HEIGHT,
      SDL_WINDOW_SHOWN);

    if (window == NULL) {
        fprintf(stderr, "ERROR: SDL failed to create window: %s\r\n", SDL_GetError());
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        fprintf(stderr, "ERROR: SDL failed to create renderer: %s\r\n", SDL_GetError());
        return false;
    }

    board_render_clear();
    board_render_commit();

    return true;
}

void board_render_deinit(void) {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

bool board_render_should_run(void) {
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
            return false;
        }
    }

    return true;
}

void board_render_clear(void) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}

void board_render_pixel(uint16_t x, uint16_t y, render_color_t color) {
    uint8_t draw_color = 0x00;
    if (color & 0x01) {
        draw_color |= 0x55;
    }
    if (color & 0x02) {
        draw_color |= 0xAA;
    }
    SDL_SetRenderDrawColor(renderer, draw_color, draw_color, draw_color, 255);
    SDL_RenderDrawPoint(renderer, x, (RENDER_HEIGHT - 1) - y);
}

void board_render_commit(void) {
    SDL_RenderPresent(renderer);
}
