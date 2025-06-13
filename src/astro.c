#include "astro.h"

#include <erfa.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define HALF_PI 1.57079632679489661923
#define VMAG_MIN -1.5
#define VMAG_MAX 12

void get_stars(
  double latitude,
  double longitude,
  struct tm const* time,
  double subsecond,
  star_callback_t on_star) {
    int ret;

    double height = 0.0;
    double proper_motion_ra = 0.0;
    double proper_motion_dec = 0.0;
    double parallax = 0.0;
    double radial_velocity = 0.0;
    double xp = 0.0;
    double yp = 0.0;
    double pressure_hpa = 0.0;
    double temperature_c = 0.0;
    double relative_humidity = 0.0;
    double obs_wavelength_microns = 1.0;
    double dut1 = 0;

    int year = time->tm_year + 1900;
    int month = time->tm_mon + 1;
    int day = time->tm_mday;
    int hour = time->tm_hour;
    int minute = time->tm_min;
    double second = time->tm_sec + subsecond;

    double utc1, utc2;
    if ((ret = eraDtf2d("UTC", year, month, day, hour, minute, second, &utc1, &utc2))) {
        fprintf(stderr, "ERROR: eraDtf2d failed with code %d\n", ret);
        return;
    }

    // stars are sorted lowest magnitude first. output highest magnitude first
    for (size_t i = BSC_SIZE; i-- > 0;) {
        double azimuth, zenith_distance;
        double observed_hour_angle, observed_dec, observed_ra, equation_of_origins;

        if ((ret = eraAtco13(
               _BSC_RA[i],
               _BSC_DEC[i],
               proper_motion_ra,
               proper_motion_dec,
               parallax,
               radial_velocity,
               utc1,
               utc2,
               dut1,
               longitude,
               latitude,
               height,
               xp,
               yp,
               pressure_hpa,
               temperature_c,
               relative_humidity,
               obs_wavelength_microns,
               &azimuth,
               &zenith_distance,
               &observed_hour_angle,
               &observed_dec,
               &observed_ra,
               &equation_of_origins))) {
            fprintf(stderr, "ERROR: eraAtco13 failed at index %zu with code %d\n", i, ret);
            continue;
        }

        double r = zenith_distance / HALF_PI;
        // filter below horizon
        if (r <= 1) {
            // compute normalized intensity
            double intensity = pow(10.0, -0.4 * _BSC_VMAG[i]);
            double intensity_min = pow(10.0, -0.4 * VMAG_MAX);
            double intensity_max = pow(10.0, -0.4 * VMAG_MIN);
            double norm = (intensity - intensity_min) / (intensity_max - intensity_min);
            norm = fmax(0.0, fmin(norm, 1.0));

            // apply an exponential scaling to emphasize "decently bright" stars
            intensity = pow(norm, 0.25);

            star_t star = {
                .x = r * sin(azimuth),
                .y = r * cos(azimuth),
                .intensity = intensity,
            };
            on_star(&star);
        }
    }
}
