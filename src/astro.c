#include "astro.h"

#include <erfa.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define HALF_PI 1.57079632679489661923f
#define VMAG_MIN -1.5f
#define VMAG_MAX 12.0f

void get_stars(
  struct tm const* time,
  float subsecond,
  float latitude,
  float longitude,
  star_callback_t on_star) {
    int ret;

    float height = 0.0;
    float proper_motion_ra = 0.0;
    float proper_motion_dec = 0.0;
    float parallax = 0.0;
    float radial_velocity = 0.0;
    float xp = 0.0;
    float yp = 0.0;
    float pressure_hpa = 0.0;
    float temperature_c = 0.0;
    float relative_humidity = 0.0;
    float obs_wavelength_microns = 1.0;
    float dut1 = 0;

    int year = time->tm_year + 1900;
    int month = time->tm_mon + 1;
    int day = time->tm_mday;
    int hour = time->tm_hour;
    int minute = time->tm_min;
    float second = time->tm_sec + subsecond;

    float utc1, utc2;
    if ((ret = eraDtf2d("UTC", year, month, day, hour, minute, second, &utc1, &utc2))) {
        fprintf(stderr, "ERROR: eraDtf2d failed with code %d\r\n", ret);
        return;
    }

    // stars are sorted lowest magnitude first. output highest magnitude first
    for (size_t i = BSC_SIZE; i-- > 0;) {
        float azimuth, zenith_distance;
        float observed_hour_angle, observed_dec, observed_ra, equation_of_origins;

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
            fprintf(stderr, "ERROR: eraAtco13 failed at index %zu with code %d\r\n", i, ret);
            continue;
        }

        float r = zenith_distance / HALF_PI;
        // filter below horizon
        if (r <= 1) {
            // compute normalized intensity
            float intensity = powf(10, -0.4f * _BSC_VMAG[i]);
            float intensity_min = powf(10, -0.4f * VMAG_MAX);
            float intensity_max = powf(10, -0.4f * VMAG_MIN);
            float intensity_normed = (intensity - intensity_min) / (intensity_max - intensity_min);
            intensity_normed = fmaxf(0, fminf(intensity_normed, 1));

            star_t star = {
                .x = -r * sinf(azimuth),
                .y = r * cosf(azimuth),
                .intensity = intensity_normed,
            };
            on_star(&star);
        }
    }
}
