#include "board_gps.h"

#include "board_pins.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>

// clang-format off
#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>
// clang-format on

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEG_TO_RAD(x) ((x) * 3.14159265358979323846 / 180.0)

static StreamBufferHandle_t uart_stream;

static double gps_latitude = 0;
static double gps_longitude = 0;

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

    while (1) {
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
                    gps_latitude = DEG_TO_RAD(*(int32_t*)(&message[state + 28]) * 1e-7);
                    gps_longitude = DEG_TO_RAD(*(int32_t*)(&message[state + 24]) * 1e-7);
                    uint16_t year = *(uint16_t*)(&message[state + 4]);
                    uint8_t month = message[state + 6];
                    uint8_t day = message[state + 7];
                    uint8_t hour = message[state + 8];
                    uint8_t minute = message[state + 9];
                    uint8_t second = message[state + 10];
                    int32_t subsecond_ns = *(int32_t*)(&message[state + 16]);
                    printf(
                      "%02d/%02d/%04d %02d:%02d:%02d %ld\r\n",
                      month,
                      day,
                      year,
                      hour,
                      minute,
                      second,
                      subsecond_ns);
                }

                state = HEADER1;
                xStreamBufferSetTriggerLevel(uart_stream, 1);
                break;
        }
    }
}

void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        char data = LL_USART_ReceiveData8(USART1);
        xStreamBufferSendFromISR(uart_stream, &data, 1, pdFALSE);
        portYIELD_FROM_ISR(pdFALSE);
    }
}

bool board_gps_init(void) {
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

    // disable NMEA output messages
    uint32_t cfg_uart1outprot_nmea_key = 0x10740002;
    uint8_t cfg_uart1outprot_nmea_value = 0;
    // enable UBX-NAV-PVT messages every 1 second
    uint32_t cfg_msgout_ubx_nav_pvt_uart1_key = 0x20910007;
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
    set_configuration_items(configuration_data, length);

    uart_stream = xStreamBufferCreate(256, 1);
    configASSERT(xTaskCreate(gps_rx_task, "gps_rx", 256, NULL, 1, NULL) == pdPASS);

    LL_USART_EnableIT_RXNE(USART1);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    return true;
}

void board_gps_deinit(void) {}

bool board_gps_location(double* latitude, double* longitude) {
    return false;
}
