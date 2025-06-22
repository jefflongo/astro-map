#include "board_render.h"

#include "board_pins.h"
#include "uc8179.h"

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_spi.h>

// clang-format off
#include <FreeRTOS.h>
#include <task.h>
// clang-format on

#include <assert.h>
#include <string.h>

#define WIDTH 648
#define HEIGHT 480

uint16_t const RENDER_WIDTH = WIDTH;
uint16_t const RENDER_HEIGHT = HEIGHT;
uint32_t const RENDER_FREQ_MS = 60 * 1000;

static uint8_t framebuffer[2][WIDTH * HEIGHT / 8];

#undef WIDTH
#undef HEIGHT

static volatile TaskHandle_t task_waiting_on_busy = NULL;

void DISPLAY_BUSY_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_EXTI_IsActiveFlag_0_31(DISPLAY_BUSY_EXTI_LINE)) {
        LL_EXTI_ClearFlag_0_31(DISPLAY_BUSY_EXTI_LINE);
        if (task_waiting_on_busy) {
            vTaskNotifyGiveFromISR(task_waiting_on_busy, &xHigherPriorityTaskWoken);
            task_waiting_on_busy = NULL;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void write_command(uint8_t command) {
    LL_GPIO_ResetOutputPin(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    LL_GPIO_ResetOutputPin(DISPLAY_SS_PORT, DISPLAY_SS_PIN);

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData8(SPI1, command);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
        ;

    LL_GPIO_SetOutputPin(DISPLAY_SS_PORT, DISPLAY_SS_PIN);
    LL_GPIO_SetOutputPin(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
}

static void write_data(uint8_t data) {
    LL_GPIO_ResetOutputPin(DISPLAY_SS_PORT, DISPLAY_SS_PIN);

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData8(SPI1, data);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
        ;

    LL_GPIO_SetOutputPin(DISPLAY_SS_PORT, DISPLAY_SS_PIN);
}

static void display_power_on(void) {
    task_waiting_on_busy = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(task_waiting_on_busy);
    ulTaskNotifyValueClear(task_waiting_on_busy, UINT32_MAX);
    write_command(UC8179_CMD_POWER_ON);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void display_power_off(void) {
    task_waiting_on_busy = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(task_waiting_on_busy);
    ulTaskNotifyValueClear(task_waiting_on_busy, UINT32_MAX);
    write_command(UC8179_CMD_POWER_OFF);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void display_refresh(void) {
    task_waiting_on_busy = xTaskGetCurrentTaskHandle();
    xTaskNotifyStateClear(task_waiting_on_busy);
    ulTaskNotifyValueClear(task_waiting_on_busy, UINT32_MAX);
    write_command(UC8179_CMD_REFRESH);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void display_init(void) {
    // reset
    vTaskDelay(pdMS_TO_TICKS(10));
    LL_GPIO_SetOutputPin(DISPLAY_RESET_PORT, DISPLAY_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(10)); // > 1ms after releasing reset

    write_command(UC8179_CMD_POWER_SETTING);
    write_data(0x07); // generate voltages internally
    write_data(0x17); // +/-20V gate voltage
    write_data(0x3F); // +15V high voltage for black/white pixel
    write_data(0x3F); // -15V low voltage for black/white pixel

    write_command(UC8179_CMD_PANEL_SETTING);
    write_data(0xBF); // B/W mode, scan up and right, enable booster

    write_command(UC8179_CMD_PLL_CTRL);
    write_data(0x06); // 50 Hz framerate

    write_command(UC8179_CMD_RESOLUTION_SETTING);
    write_data((uint8_t)(RENDER_WIDTH >> 8));
    write_data((uint8_t)(RENDER_WIDTH & 0xFF));
    write_data((uint8_t)(RENDER_HEIGHT >> 8));
    write_data((uint8_t)(RENDER_HEIGHT & 0xFF));

    write_command(UC8179_CMD_DUAL_SPI);
    write_data(0x00); // disable dual SPI

    write_command(UC8179_CMD_TCON_SETTING);
    write_data(0x22); // use default S2G/G2G periods

    write_command(UC8179_CMD_VCOM_DC_SETTING);
    write_data(0x12); // VCOM -1V

    write_command(UC8179_CMD_VCOM_DATA_INT_SETTING);
    write_data(0x21); // configure data polarity (0 -> black, 1 -> white)
    write_data(0x07); // use default VCOM/data interval

    uint8_t const lut_vcom[] = {
        0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, // state 1
        0x60, 0x14, 0x14, 0x00, 0x00, 0x01, // state 2
        0x00, 0x14, 0x00, 0x00, 0x00, 0x01, // state 3
        0x00, 0x13, 0x0A, 0x01, 0x00, 0x01, // state 4
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 6
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 7
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 8  (KWR mode only)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 9  (KWR mode only)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 10 (KWR mode only)
    };

    uint8_t const lut_ww[] = {
        0x40, 0x0A, 0x00, 0x00, 0x00, 0x01, // state 1
        0x90, 0x14, 0x14, 0x00, 0x00, 0x01, // state 2
        0x10, 0x14, 0x0A, 0x00, 0x00, 0x01, // state 3
        0xA0, 0x13, 0x01, 0x00, 0x00, 0x01, // state 4
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 6
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 7
    };

    uint8_t const lut_bw[] = {
        0x40, 0x0A, 0x00, 0x00, 0x00, 0x01, // state 1
        0x90, 0x14, 0x14, 0x00, 0x00, 0x01, // state 2
        0x00, 0x14, 0x0A, 0x00, 0x00, 0x01, // state 3
        0x99, 0x0C, 0x01, 0x03, 0x04, 0x01, // state 4
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 6
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 7
    };

    uint8_t const lut_wb[] = {
        0x40, 0x0A, 0x00, 0x00, 0x00, 0x01, // state 1
        0x90, 0x14, 0x14, 0x00, 0x00, 0x01, // state 2
        0x00, 0x14, 0x0A, 0x00, 0x00, 0x01, // state 3
        0x99, 0x0B, 0x04, 0x04, 0x01, 0x01, // state 4
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 6
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 7
    };

    uint8_t const lut_bb[] = {
        0x80, 0x0A, 0x00, 0x00, 0x00, 0x01, // state 1
        0x90, 0x14, 0x14, 0x00, 0x00, 0x01, // state 2
        0x20, 0x14, 0x0A, 0x00, 0x00, 0x01, // state 3
        0x50, 0x13, 0x01, 0x00, 0x00, 0x01, // state 4
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 6
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // state 7
    };

    // load LUTs
    write_command(UC8179_CMD_VCOM_LUT);
    for (size_t i = 0; i < sizeof(lut_vcom); i++) {
        write_data(lut_vcom[i]);
    }

    write_command(UC8179_CMD_W2W_LUT);
    for (size_t i = 0; i < sizeof(lut_ww); i++) {
        write_data(lut_ww[i]);
    }

    write_command(UC8179_CMD_K2W_LUT);
    for (size_t i = 0; i < sizeof(lut_bw); i++) {
        write_data(lut_bw[i]);
    }

    write_command(UC8179_CMD_W2K_LUT);
    for (size_t i = 0; i < sizeof(lut_wb); i++) {
        write_data(lut_wb[i]);
    }

    write_command(UC8179_CMD_K2K_LUT);
    for (size_t i = 0; i < sizeof(lut_bb); i++) {
        write_data(lut_bb[i]);
    }

    write_command(UC8179_CMD_BORDER_LUT);
    for (size_t i = 0; i < sizeof(lut_ww); i++) {
        write_data(lut_ww[i]);
    }
}

bool board_render_init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_GPIO_SetAFPin_0_7(DISPLAY_MOSI_PORT, DISPLAY_MOSI_PIN, LL_GPIO_AF_5);
    LL_GPIO_SetPinSpeed(DISPLAY_MOSI_PORT, DISPLAY_MOSI_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(DISPLAY_MOSI_PORT, DISPLAY_MOSI_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(DISPLAY_MOSI_PORT, DISPLAY_MOSI_PIN, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(DISPLAY_MISO_PORT, DISPLAY_MISO_PIN, LL_GPIO_AF_5);
    LL_GPIO_SetPinSpeed(DISPLAY_MISO_PORT, DISPLAY_MISO_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(DISPLAY_MISO_PORT, DISPLAY_MISO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(DISPLAY_MISO_PORT, DISPLAY_MISO_PIN, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(DISPLAY_SCK_PORT, DISPLAY_SCK_PIN, LL_GPIO_AF_5);
    LL_GPIO_SetPinSpeed(DISPLAY_SCK_PORT, DISPLAY_SCK_PIN, LL_GPIO_SPEED_FREQ_MEDIUM);
    LL_GPIO_SetPinOutputType(DISPLAY_SCK_PORT, DISPLAY_SCK_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinMode(DISPLAY_SCK_PORT, DISPLAY_SCK_PIN, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetOutputPin(DISPLAY_SS_PORT, DISPLAY_SS_PIN);
    LL_GPIO_SetPinMode(DISPLAY_SS_PORT, DISPLAY_SS_PIN, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_ResetOutputPin(DISPLAY_RESET_PORT, DISPLAY_RESET_PIN);
    LL_GPIO_SetPinMode(DISPLAY_RESET_PORT, DISPLAY_RESET_PIN, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_ResetOutputPin(DISPLAY_DC_PORT, DISPLAY_DC_PIN);
    LL_GPIO_SetPinMode(DISPLAY_DC_PORT, DISPLAY_DC_PIN, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetPinPull(DISPLAY_BUSY_PORT, DISPLAY_BUSY_PIN, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinMode(DISPLAY_BUSY_PORT, DISPLAY_BUSY_PIN, LL_GPIO_MODE_INPUT);

    LL_SYSCFG_SetEXTISource(DISPLAY_BUSY_SYSCFG_EXTI_PORT, DISPLAY_BUSY_SYSCFG_EXTI_LINE);
    LL_EXTI_InitTypeDef conn_irq_conf = {
        .Line_0_31 = DISPLAY_BUSY_EXTI_LINE,
        .Line_32_63 = 0,
        .LineCommand = ENABLE,
        .Mode = LL_EXTI_MODE_IT,
        .Trigger = LL_EXTI_TRIGGER_RISING,
    };
    LL_EXTI_Init(&conn_irq_conf);

    LL_SPI_InitTypeDef spi_config = {
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode = LL_SPI_MODE_MASTER,
        .DataWidth = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity = LL_SPI_POLARITY_LOW,
        .ClockPhase = LL_SPI_PHASE_1EDGE,
        .NSS = LL_SPI_NSS_SOFT,
        .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256, // 6.66MHz max
        .BitOrder = LL_SPI_MSB_FIRST,
        .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
    };

    LL_SPI_Init(SPI1, &spi_config);
    LL_SPI_EnableNSSPulseMgt(SPI1);
    LL_SPI_Enable(SPI1);

    display_init();

    NVIC_SetPriority(DISPLAY_BUSY_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 7, 0));
    NVIC_EnableIRQ(DISPLAY_BUSY_IRQn);

    board_render_clear();
    board_render_commit();

    return true;
}

void board_render_deinit(void) {}

bool board_render_should_run(void) {
    return true;
}

void board_render_clear(void) {
    memset(framebuffer, 0, sizeof(framebuffer));
}

void board_render_pixel(uint16_t x, uint16_t y, render_color_t color) {
    size_t bit_index = y * RENDER_WIDTH + x;
    size_t byte_index = bit_index / 8;
    size_t bit_offset = bit_index % 8;
    assert(byte_index < sizeof(framebuffer[0]));

    if (color & 0x02) {
        framebuffer[0][byte_index] |= (1 << bit_offset);
    } else {
        framebuffer[0][byte_index] &= ~(1 << bit_offset);
    }

    if (color & 0x01) {
        framebuffer[1][byte_index] |= (1 << bit_offset);
    } else {
        framebuffer[1][byte_index] &= ~(1 << bit_offset);
    }
}

void board_render_commit(void) {
    display_power_on();

    write_command(UC8179_CMD_DATA_TX1);
    for (size_t i = 0; i < sizeof(framebuffer[0]); i++) {
        write_data(framebuffer[0][i]);
    }

    write_command(UC8179_CMD_DATA_TX2);
    for (size_t i = 0; i < sizeof(framebuffer[1]); i++) {
        write_data(framebuffer[1][i]);
    }

    display_refresh();
    display_power_off();
}
