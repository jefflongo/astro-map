#include "astro.h"
#include "board.h"
#include "render.h"

// clang-format off
#include <FreeRTOS.h>
#include <task.h>
// clang-format on

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static void main_task(void* args) {
    (void)args;

    bool init_success = board_gps_init() && board_render_init();

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t const frequency = pdMS_TO_TICKS(RENDER_FREQ_MS);

    bool run = init_success;
    while (run) {
        // get time/location
        struct tm time;
        float subsecond, latitude, longitude;
        if (board_gps_time_location(&time, &subsecond, &latitude, &longitude)) {
            // get stars at location / time
            board_render_clear();
            get_stars(&time, subsecond, latitude, longitude, render_star);
            printf("Updating display\r\n");
            board_render_commit();
        }

        if ((run = board_render_should_run())) {
            vTaskDelayUntil(&last_wake, frequency);
        }
    }

    board_gps_deinit();
    board_render_deinit();

    exit(init_success ? 0 : 1);
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    assert(board_init());
    xTaskCreate(main_task, "main", 512, NULL, 1, NULL);
    vTaskStartScheduler();

    // unreachable
    while (1)
        ;

    return 0;
}
