#include "board_gps.h"

// clang-format off
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>
#include <task.h>
// clang-format on

#include <curl/curl.h>
#include <stdint.h>
#include <string.h>

#define GPS_TASK_FREQ_MS (10 * 60 * 1000)

#define DEG_TO_RAD(x) ((x) * 3.14159265358979323846f / 180.0f)

static CURL* curl = NULL;
static char curl_buffer[32];
static size_t curl_pos = 0;

static EventGroupHandle_t gps_events = NULL;
static SemaphoreHandle_t gps_mutex = NULL;

static float gps_latitude = 0;
static float gps_longitude = 0;

static size_t write_callback(void* ptr, size_t size, size_t nmemb, void* userdata) {
    (void)userdata;

    size_t len = size * nmemb;
    if (curl_pos + len < sizeof(curl_buffer) - 1) {
        memcpy(curl_buffer + curl_pos, ptr, len);
        curl_pos += len;
        curl_buffer[curl_pos] = '\0';
    }
    return size * nmemb;
}

static void gps_rx_task(void* args) {
    (void)args;

    curl_easy_setopt(curl, CURLOPT_URL, "https://ipinfo.io/loc");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1);

    while (1) {
        bool success = true;
        float latitude, longitude;

        CURLcode res = curl_easy_perform(curl);
        if (success && !(success = (res == CURLE_OK))) {
            fprintf(stderr, "ERROR: curl_easy_perform() failed: %s\r\n", curl_easy_strerror(res));
        }

        if (success && !(success = sscanf(curl_buffer, "%f,%f", &latitude, &longitude) == 2)) {
            fprintf(stderr, "ERROR: failed to parse location: \"%s\"\r\n", curl_buffer);
        }

        if (success) {
            xEventGroupSetBits(gps_events, 1);
            xSemaphoreTake(gps_mutex, portMAX_DELAY);
            gps_latitude = DEG_TO_RAD(latitude);
            gps_longitude = DEG_TO_RAD(longitude);
            xSemaphoreGive(gps_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_FREQ_MS));
    }
}

bool board_gps_init(void) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (!curl) {
        return false;
    }

    gps_events = xEventGroupCreate();
    gps_mutex = xSemaphoreCreateMutex();
    xTaskCreate(gps_rx_task, "gps_rx", 256, NULL, 2, NULL);

    return true;
}

void board_gps_deinit(void) {
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

bool board_gps_time_location(struct tm* time, float* subsecond, float* latitude, float* longitude) {
    // block until GPS valid
    if (!(xEventGroupWaitBits(gps_events, 1, pdFALSE, pdTRUE, pdMS_TO_TICKS(1000)) & 1)) {
        return false;
    }

    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    struct tm* now = gmtime(&ts.tv_sec);

    *time = *now;
    *subsecond = ts.tv_nsec / 1e9f;

    xSemaphoreTake(gps_mutex, portMAX_DELAY);
    *latitude = gps_latitude;
    *longitude = gps_longitude;
    xSemaphoreGive(gps_mutex);

    return true;
}
