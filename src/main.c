#include "astro.h"
#include "board.h"
#include "render.h"

#include <time.h>

#define VMAG_MAX 6

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    bool init_success = true;
    init_success = init_success && board_gps_init();
    init_success = init_success && board_render_init();
    if (!init_success) {
        goto cleanup;
    }

    bool run = true;
    while (run) {
        // get location
        double latitude, longitude;
        board_gps_get_location(&latitude, &longitude);

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
        run = board_render_delay();
    }

cleanup:
    board_gps_deinit();
    board_render_deinit();

    return init_success ? 0 : 1;
}
