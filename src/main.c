#include "astro.h"
#include "board.h"
#include "render.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <time.h>

#define VMAG_MAX 6

static void main_task(void* args) {
    (void)args;

    bool init_success = board_init();
    init_success = init_success && board_gps_init();
    init_success = init_success && board_render_init();

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t frequency = pdMS_TO_TICKS(RENDER_FREQ_MS);

    bool run = init_success;
    while (run) {
        // get location
        double latitude, longitude;
        board_gps_location(&latitude, &longitude);

        // get time
        time_t now = time(NULL);
        struct tm* time = gmtime(&now);
        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        double subsecond = ts.tv_nsec / 1e9;

        // get stars at location/time
        static star_t stars[COORD_BUFFER_SIZE];
        size_t n_stars_above_horizon;
        if (get_stars(
              latitude, longitude, time, subsecond, VMAG_MAX, stars, &n_stars_above_horizon)) {
            render_stars(stars, n_stars_above_horizon);
        }
        if ((run = board_render_should_run())) {
            vTaskDelayUntil(&last_wake, frequency);
        }
    }

    // unreachable on embedded target
    board_gps_deinit();
    board_render_deinit();

    exit(init_success ? 0 : 1);
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    xTaskCreate(main_task, "main_task", 4096, NULL, 1, NULL);
    vTaskStartScheduler();

    // unreachable
    while (1)
        ;

    return 0;
}
