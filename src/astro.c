#include "astro.h"

#include <erfa.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define HALF_PI 1.57079632679489661923
#define VMAG_MIN -1.5
#define VMAG_MAX 12

static int compare_stars(void const* p_a, void const* p_b) {
    double ia = ((star_t const*)p_a)->intensity;
    double ib = ((star_t const*)p_b)->intensity;
    return (ia > ib) - (ia < ib);
}

bool get_stars(
  double longitude,
  double latitude,
  struct tm const* time,
  double subsecond,
  double vmag_max,
  star_t out[COORD_BUFFER_SIZE],
  size_t* out_size) {
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

    *out_size = 0;

    double utc1, utc2;
    if ((ret = eraDtf2d("UTC", year, month, day, hour, minute, second, &utc1, &utc2))) {
        fprintf(stderr, "ERROR: eraDtf2d returned %d\n", ret);
        return false;
    }

    for (size_t i = 0; i < BSC_SIZE; i++) {
        if (_BSC_VMAG[i] > vmag_max) {
            continue;
        }

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
            fprintf(stderr, "ERROR: eraAtco13 returned %d at index %zu\n", ret, i);
            return false;
        }

        double r = zenith_distance / HALF_PI;
        // filter below horizon
        if (r <= 1) {
            size_t j = (*out_size)++;
            out[j].x = -r * sin(azimuth);
            out[j].y = r * cos(azimuth);

            // compute normalized intensity
            double intensity = pow(10.0, -0.4 * _BSC_VMAG[i]);
            double intensity_min = pow(10.0, -0.4 * VMAG_MAX);
            double intensity_max = pow(10.0, -0.4 * VMAG_MIN);
            double norm = (intensity - intensity_min) / (intensity_max - intensity_min);
            norm = fmax(0.0, fmin(norm, 1.0));

            // apply an exponential scaling to emphasize "decently bright" stars
            out[j].intensity = pow(norm, 0.25);
        }
    }

    // sort by ascending intensity
    qsort(out, *out_size, sizeof(star_t), compare_stars);

    return true;
}
