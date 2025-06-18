#include "board_gps.h"

#include "board_pins.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_rtc.h>
#include <stm32g4xx_ll_usart.h>

// clang-format off
#include <FreeRTOS.h>
#include <semphr.h>
#include <stream_buffer.h>
#include <task.h>
// clang-format on

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define GPS_TASK_FREQ_MS (10 * 60 * 1000)

#define RTC_VALID_MAGIC 0x32F2

#define RTC_PREDIV_S 255
#define RTC_PREDIV_A 127
static_assert((RTC_PREDIV_S + 1) * (RTC_PREDIV_A + 1) == 32768);

#define DEG_TO_RAD(x) ((x) * 3.14159265358979323846 / 180.0)

static StreamBufferHandle_t uart_stream;
static SemaphoreHandle_t gps_mutex = NULL;

static double gps_latitude = 0;
static double gps_longitude = 0;

static void gps_rx_task(void* args) {
    (void)args;

    enum {
        HEADER1 = 0,
        HEADER2,
        CLASS,
        ID,
        LEN1,
        LEN2,
        PAYLOAD_AND_CHECKSUM,
    } state = HEADER1;

    uint8_t message[100]; // UBX-NAV-PVT message size
    size_t const payload_size = sizeof(message) - 8;
    uint8_t ck_a, ck_b;

    bool backup_regs_valid = LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR0) == RTC_VALID_MAGIC;

    while (1) {
        bool time_location_updated = false;

        xStreamBufferReceive(
          uart_stream,
          &message[state],
          state == PAYLOAD_AND_CHECKSUM ? (payload_size + 2) : 1,
          portMAX_DELAY);

        switch (state) {
            case HEADER1:
                if (message[state] == 0xb5) {
                    state = HEADER2;
                }
                break;

            case HEADER2:
                if (message[state] == 0x62) {
                    state = CLASS;
                } else {
                    state = HEADER1;
                }
                break;

            case CLASS:
                if (message[state] == 0x01) {
                    ck_a = message[state];
                    ck_b = ck_a;
                    state = ID;
                } else {
                    state = HEADER1;
                }
                break;

            case ID:
                if (message[state] == 0x07) {
                    ck_a += message[state];
                    ck_b += ck_a;
                    state = LEN1;
                } else {
                    state = HEADER1;
                }
                break;

            case LEN1:
                if (message[state] == payload_size) {
                    ck_a += message[state];
                    ck_b += ck_a;
                    state = LEN2;
                } else {
                    state = HEADER1;
                }
                break;

            case LEN2:
                if (message[state] == 0) {
                    ck_a += message[state];
                    ck_b += ck_a;
                    state = PAYLOAD_AND_CHECKSUM;
                    xStreamBufferSetTriggerLevel(uart_stream, payload_size + 2);
                } else {
                    state = HEADER1;
                }
                break;

            case PAYLOAD_AND_CHECKSUM:
                for (uint16_t i = state; i < sizeof(message) - 2; i++) {
                    ck_a += message[i];
                    ck_b += ck_a;
                }
                bool valid =
                  message[sizeof(message) - 2] == ck_a && message[sizeof(message) - 1] == ck_b;

                // ensure date and time are valid and fully resolved
                valid = valid && ((message[state + 11] & 0x07) == 0x07);
                // ensure valid fix
                valid = valid && (message[state + 21] & 1);
                // require 2d fix, 3d fix, or GNSS + dead reckoning combined
                valid = valid && (message[state + 20] > 1 && message[state + 20] < 5);
                // confirm time and date
                valid = valid && ((message[state + 22] & 0xE0) == 0xE0);

                if (valid) {
                    uint8_t hour = message[state + 8];
                    uint8_t minute = message[state + 9];
                    uint8_t second = message[state + 10];
                    uint8_t month = message[state + 6];
                    uint8_t day = message[state + 7];
                    uint8_t year = *(uint16_t*)(&message[state + 4]) - 2000;

                    double latitude = DEG_TO_RAD(*(int32_t*)(&message[state + 28]) * 1e-7);
                    double longitude = DEG_TO_RAD(*(int32_t*)(&message[state + 24]) * 1e-7);

                    xSemaphoreTake(gps_mutex, portMAX_DELAY);

                    // update location
                    gps_latitude = latitude;
                    gps_longitude = longitude;

                    // update RTC
                    LL_RTC_DisableWriteProtection(RTC);
                    if (LL_RTC_EnterInitMode(RTC) == SUCCESS) {
                        LL_RTC_TIME_Config(
                          RTC,
                          LL_RTC_TIME_FORMAT_AM_OR_24,
                          __LL_RTC_CONVERT_BIN2BCD(hour),
                          __LL_RTC_CONVERT_BIN2BCD(minute),
                          __LL_RTC_CONVERT_BIN2BCD(second));

                        LL_RTC_DATE_Config(
                          RTC,
                          LL_RTC_WEEKDAY_MONDAY, // don't care
                          __LL_RTC_CONVERT_BIN2BCD(day),
                          __LL_RTC_CONVERT_BIN2BCD(month),
                          __LL_RTC_CONVERT_BIN2BCD(year));

                        time_location_updated = true;
                    }
                    LL_RTC_DisableInitMode(RTC);
                    LL_RTC_WaitForSynchro(RTC);
                    LL_RTC_EnableWriteProtection(RTC);

                    xSemaphoreGive(gps_mutex);

                    // store location into backup registers
                    uint64_t latitude_bits;
                    memcpy(&latitude_bits, &gps_latitude, sizeof(double));
                    uint32_t latitude_low = (uint32_t)(latitude_bits & 0xFFFFFFFF);
                    uint32_t latitude_high = (uint32_t)(latitude_bits >> 32);
                    LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR1, latitude_low);
                    LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR2, latitude_high);

                    uint64_t longitude_bits;
                    memcpy(&longitude_bits, &gps_longitude, sizeof(double));
                    uint32_t longitude_low = (uint32_t)(longitude_bits & 0xFFFFFFFF);
                    uint32_t longitude_high = (uint32_t)(longitude_bits >> 32);
                    LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR3, longitude_low);
                    LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR4, longitude_high);

                    if (!backup_regs_valid) {
                        LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR0, RTC_VALID_MAGIC);
                        backup_regs_valid = true;
                    }
                }

                state = HEADER1;
                xStreamBufferSetTriggerLevel(uart_stream, 1);
                break;
        }

        // wait before listening to the GPS again
        if (time_location_updated) {
            LL_USART_DisableIT_RXNE(USART1);
            xStreamBufferReset(uart_stream);
            vTaskDelay(pdMS_TO_TICKS(GPS_TASK_FREQ_MS));
            LL_USART_EnableIT_RXNE(USART1);
        }
    }
}

void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        char data = LL_USART_ReceiveData8(USART1);
        xStreamBufferSendFromISR(uart_stream, &data, 1, pdFALSE);
    }

    portYIELD_FROM_ISR(pdFALSE);
}

static void rtc_init(void) {
    // TODO: need to use LSE for VBAT backup to work.
    // clock source cannot be changed without resetting RTC.
    if (LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI) {
        LL_RCC_ForceBackupDomainReset();
        LL_RCC_ReleaseBackupDomainReset();
        LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
    }
    LL_RCC_EnableRTC();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTCAPB);

    LL_RTC_InitTypeDef rtc_config = {
        .HourFormat = LL_RTC_HOURFORMAT_24HOUR,
        // prescale 32.768 kHz input oscillator to 1 Hz
        .AsynchPrescaler = RTC_PREDIV_A,
        .SynchPrescaler = RTC_PREDIV_S,
    };
    LL_RTC_Init(RTC, &rtc_config);

    if (LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR0) == RTC_VALID_MAGIC) {
        // load location from backup
        uint32_t latitude_low = LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR1);
        uint32_t latitude_high = LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR2);
        uint64_t latitude_bits = ((uint64_t)latitude_high << 32) | latitude_low;
        memcpy(&gps_latitude, &latitude_bits, sizeof(double));

        uint32_t longitude_low = LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR3);
        uint32_t longitude_high = LL_RTC_BKP_GetRegister(RTC, LL_RTC_BKP_DR4);
        uint64_t longitude_bits = ((uint64_t)longitude_high << 32) | longitude_low;
        memcpy(&gps_longitude, &longitude_bits, sizeof(double));
    }
}

static void uart_init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_GPIO_SetAFPin_8_15(GPS_TX_PORT, GPS_TX_PIN, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPS_TX_PORT, GPS_TX_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(GPS_TX_PORT, GPS_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(GPS_TX_PORT, GPS_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(GPS_TX_PORT, GPS_TX_PIN, LL_GPIO_PULL_UP);

    LL_GPIO_SetAFPin_8_15(GPS_RX_PORT, GPS_RX_PIN, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPS_RX_PORT, GPS_RX_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinMode(GPS_RX_PORT, GPS_RX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(GPS_RX_PORT, GPS_RX_PIN, LL_GPIO_PULL_UP);

    LL_USART_InitTypeDef uart_config = {
        .BaudRate = 9600,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .TransferDirection = LL_USART_DIRECTION_TX_RX,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    };
    LL_USART_Init(USART1, &uart_config);
    LL_USART_DisableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_Enable(USART1);

    while (!LL_USART_IsActiveFlag_TEACK(USART1) || !(LL_USART_IsActiveFlag_REACK(USART1)))
        ;
}

static bool set_configuration_items(void const* configuration_data, uint16_t len) {
    // create a UBX-CFG-VALSET message
    uint16_t message_length = 12 + len;
    uint8_t message[12 + len];

    message[0] = 0xb5;                      // header 1
    message[1] = 0x62;                      // header 2
    message[2] = 0x06;                      // class
    message[3] = 0x8a;                      // id
    message[4] = (uint8_t)(4 + len);        // length LSB
    message[5] = (uint8_t)((4 + len) >> 8); // length MSB
    message[6] = 0x00;                      // version
    message[7] = 0x01;                      // layers (RAM)
    message[8] = 0x00;                      // reserved 1
    message[9] = 0x00;                      // reserved 2
    memcpy(&message[10], configuration_data, len);

    // message checksum (skip header)
    message[message_length - 2] = message[message_length - 1] = 0;
    for (uint16_t i = 2; i < message_length - 2; i++) {
        message[message_length - 2] += message[i];
        message[message_length - 1] += message[message_length - 2];
    }

    // transmit message
    for (uint16_t i = 0; i < message_length; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1))
            ;
        LL_USART_TransmitData8(USART1, message[i]);
    }

    // receive a UBX-ACK-ACK/NACK response
    uint8_t response[10];
    for (uint8_t i = 0; i < 10; i++) {
        while (!LL_USART_IsActiveFlag_RXNE(USART1))
            ;
        response[i] = LL_USART_ReceiveData8(USART1);
    }
    // response checksum (skip header)
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < 10 - 2; i++) {
        ck_a += response[i];
        ck_b += ck_a;
    }

    // check for valid UBX-ACK-ACK
    bool valid = response[0] == 0xb5 && response[1] == 0x62;     // check header
    valid = valid && response[2] == 0x05;                        // check class
    valid = valid && response[3] == 0x01;                        // check id
    valid = valid && response[4] == 2 && response[5] == 0;       // check length
    valid = valid && response[6] == 0x06;                        // check acked class
    valid = valid && response[7] == 0x8a;                        // check acked id
    valid = valid && response[8] == ck_a && response[9] == ck_b; // check checksum

    return valid;
}

static bool gps_configure(void) {
    // disable NMEA output messages
    uint32_t const cfg_uart1outprot_nmea_key = 0x10740002;
    uint8_t cfg_uart1outprot_nmea_value = 0;
    // enable UBX-NAV-PVT messages every 1 second
    uint32_t const cfg_msgout_ubx_nav_pvt_uart1_key = 0x20910007;
    uint8_t cfg_msgout_ubx_nav_pvt_uart1_value = 1;

    uint8_t configuration_data[2 * (sizeof(uint32_t) + sizeof(uint8_t))];
    uint16_t length = 0;

    memcpy(configuration_data + length, &cfg_uart1outprot_nmea_key, sizeof(uint32_t));
    length += sizeof(uint32_t);
    memcpy(configuration_data + length, &cfg_uart1outprot_nmea_value, sizeof(uint8_t));
    length += sizeof(uint8_t);
    memcpy(configuration_data + length, &cfg_msgout_ubx_nav_pvt_uart1_key, sizeof(uint32_t));
    length += sizeof(uint32_t);
    memcpy(configuration_data + length, &cfg_msgout_ubx_nav_pvt_uart1_value, sizeof(uint8_t));
    length += sizeof(uint8_t);

    return set_configuration_items(configuration_data, length);
}

bool board_gps_init(void) {
    // initialize interfaces
    rtc_init();
    uart_init();
    bool ret = gps_configure();

    // set up processing task
    uart_stream = xStreamBufferCreate(256, 1);
    gps_mutex = xSemaphoreCreateMutex();
    xTaskCreate(gps_rx_task, "gps_rx", 256, NULL, 1, NULL);

    // kick off UART RX
    LL_USART_EnableIT_RXNE(USART1);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    return ret;
}

void board_gps_deinit(void) {}

bool board_gps_time_location(
  struct tm* time, double* subsecond, double* latitude, double* longitude) {
    xSemaphoreTake(gps_mutex, portMAX_DELAY);

    // perform a coherent read of the RTC registers, carefully handling cases where a read is
    // performed during a rollover
    uint32_t tr, tr_check, dr, dr_check, ssr;
    do {
        tr = LL_RTC_TIME_Get(RTC);
        ssr = LL_RTC_TIME_GetSubSecond(RTC);
        dr = LL_RTC_DATE_Get(RTC);

        tr_check = LL_RTC_TIME_Get(RTC);
        dr_check = LL_RTC_DATE_Get(RTC);
    } while ((tr != tr_check) || (dr != dr_check));

    time->tm_year = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_YEAR(dr)) + 100;
    time->tm_mon = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_MONTH(dr)) - 1;
    time->tm_mday = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_DAY(dr));
    time->tm_hour = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_HOUR(tr));
    time->tm_min = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_MINUTE(tr));
    time->tm_sec = __LL_RTC_CONVERT_BCD2BIN(__LL_RTC_GET_SECOND(tr));
    *subsecond = (RTC_PREDIV_S - ssr) / (RTC_PREDIV_S + 1);

    *latitude = gps_latitude;
    *longitude = gps_longitude;

    xSemaphoreGive(gps_mutex);

    return true;
}
