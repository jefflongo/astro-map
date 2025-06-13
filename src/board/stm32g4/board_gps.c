#include "board_gps.h"

#include "board_pins.h"

#include <stdio.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>

void USART1_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        char data = LL_USART_ReceiveData8(USART1);
        putchar(data);
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

    LL_USART_EnableIT_RXNE(USART1);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    while (!LL_USART_IsActiveFlag_TEACK(USART1) || !(LL_USART_IsActiveFlag_REACK(USART1)))
        ;

    return true;
}

void board_gps_get_location(double* latitude, double* longitude) {}
