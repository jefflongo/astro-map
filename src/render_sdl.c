#include "render.h"

#include <SDL2/SDL.h>

uint16_t const SCREEN_WIDTH = 640;
uint16_t const SCREEN_HEIGHT = 640;

static SDL_Renderer* renderer = NULL;
static SDL_Window* window = NULL;

bool render_init(void) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL failed to initialize: %s\n", SDL_GetError());
        return false;
    }

    window = SDL_CreateWindow(
      "Sky Map",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      SCREEN_WIDTH,
      SCREEN_HEIGHT,
      SDL_WINDOW_SHOWN);

    if (window == NULL) {
        fprintf(stderr, "SDL failed to create window: %s\n", SDL_GetError());
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        fprintf(stderr, "SDL failed to create renderer: %s\n", SDL_GetError());
        return false;
    }

    _render_clear();
    _render_commit();

    return true;
}

void render_deinit(void) {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

bool render_loop(void) {
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
            return false;
        }
    }
    SDL_Delay(16);

    return true;
}

void _render_clear(void) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}

void _render_pixel(uint16_t x, uint16_t y, render_color_t color) {
    SDL_SetRenderDrawColor(renderer, color, color, color, 255);
    SDL_RenderDrawPoint(renderer, x, y);
}

void _render_commit(void) {
    SDL_RenderPresent(renderer);
}
