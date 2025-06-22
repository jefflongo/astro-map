#include "board_gps.h"

#include <curl/curl.h>
#include <stdint.h>
#include <string.h>

#define DEG_TO_RAD(x) ((x) * 3.14159265358979323846f / 180.0f)

static CURL* curl = NULL;
static char curl_buffer[32];
static size_t curl_pos = 0;

static float gps_latitude = 0;
static float gps_longitude = 0;

size_t write_callback(void* ptr, size_t size, size_t nmemb, void* userdata) {
    (void)userdata;

    size_t len = size * nmemb;
    if (curl_pos + len < sizeof(curl_buffer) - 1) {
        memcpy(curl_buffer + curl_pos, ptr, len);
        curl_pos += len;
        curl_buffer[curl_pos] = '\0';
    }
    return size * nmemb;
}

bool board_gps_init(void) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (!curl) {
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, "https://ipinfo.io/loc");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1);

    return true;
}

void board_gps_deinit(void) {
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

bool board_gps_time_location(struct tm* time, float* subsecond, float* latitude, float* longitude) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    struct tm* now = gmtime(&ts.tv_sec);

    *time = *now;
    *subsecond = ts.tv_nsec / 1e9f;

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        fprintf(stderr, "ERROR: curl_easy_perform() failed: %s\r\n", curl_easy_strerror(res));
        return false;
    }

    if (sscanf(curl_buffer, "%f,%f", &gps_latitude, &gps_longitude) != 2) {
        fprintf(stderr, "ERROR: failed to parse location: \"%s\"\r\n", curl_buffer);
        return false;
    }

    *latitude = DEG_TO_RAD(gps_latitude);
    *longitude = DEG_TO_RAD(gps_longitude);

    return true;
}
