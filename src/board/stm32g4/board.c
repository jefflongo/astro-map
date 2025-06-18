#include "board.h"

#include "board_pins.h"
#include "platform.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_utils.h>

// redirect printf
int _write(int fd, char const* buf, int count) {
    for (int i = 0; i < count; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2))
            ;
        LL_USART_TransmitData8(USART2, buf[i]);
    }
    return count;
}

static void board_debug_init(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_GPIO_SetAFPin_0_7(DEBUG_TX_PORT, DEBUG_TX_PIN, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(DEBUG_TX_PORT, DEBUG_TX_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(DEBUG_TX_PORT, DEBUG_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinMode(DEBUG_TX_PORT, DEBUG_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(DEBUG_TX_PORT, DEBUG_TX_PIN, LL_GPIO_PULL_UP);

    LL_USART_InitTypeDef uart_config = {
        .BaudRate = 115200,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .TransferDirection = LL_USART_DIRECTION_TX,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    };
    LL_USART_Init(USART2, &uart_config);
    LL_USART_ConfigAsyncMode(USART2);
    LL_USART_Enable(USART2);

    while (!LL_USART_IsActiveFlag_TEACK(USART2))
        ;
}

bool board_init(void) {
    platform_init();

    // enable gpio clocks
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    board_debug_init();
    __enable_irq();

    return true;
}
