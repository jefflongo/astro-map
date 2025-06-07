#include "astro.h"
#include "render.h"

#include <stdio.h>
#include <time.h>

#define LATITUDE 0.6041190641627
#define LONGITUDE -2.0977752641
#define VMAG_MAX 6

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    if (!render_init()) {
        render_deinit();
        return 1;
    }

    bool run = true;
    while (run) {
        static star_t stars[COORD_BUFFER_SIZE];
        size_t n_stars_above_horizon;

        time_t now = time(NULL);
        struct tm* time = gmtime(&now);
        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        double subsecond = ts.tv_nsec / 1e9;

        if (get_stars(
              LONGITUDE, LATITUDE, time, subsecond, VMAG_MAX, stars, &n_stars_above_horizon)) {
            render_stars(stars, n_stars_above_horizon);
        }
        run = render_loop();
    }

    return 0;
}
