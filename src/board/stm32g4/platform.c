#include "platform.h"

#include <assert.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_pwr.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>

#ifndef NVIC_PRIORITYGROUP_0
// 0 bits for pre-emption priority 4 bits for subpriority
#define NVIC_PRIORITYGROUP_0 ((uint32_t)0x00000007)
// 1 bits for pre-emption priority 3 bits for subpriority
#define NVIC_PRIORITYGROUP_1 ((uint32_t)0x00000006)
// 2 bits for pre-emption priority 2 bits for subpriority
#define NVIC_PRIORITYGROUP_2 ((uint32_t)0x00000005)
// 3 bits for pre-emption priority 1 bits for subpriority
#define NVIC_PRIORITYGROUP_3 ((uint32_t)0x00000004)
// 4 bits for pre-emption priority 0 bits for subpriority
#define NVIC_PRIORITYGROUP_4 ((uint32_t)0x00000003)
#endif

#define HANDLER(thing, action)                                                                     \
    void thing##_Handler(void);                                                                    \
    void thing##_Handler(void) {                                                                   \
        action;                                                                                    \
    }

HANDLER(HardFault, while (1))
HANDLER(MemManage, while (1))
HANDLER(BusFault, while (1))
HANDLER(UsageFault, while (1))

HANDLER(DebugMon, )

void platform_init(void) {
    // enable system and power control clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // disable internal pull-up for type C dead battery pins
    LL_PWR_DisableUCPDDeadBattery();

    // 170 MHz -> 4 wait states (RM0440 Table 9)
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
        ;
    // 170 MHz -> range 1 boost (RM0440 Table 50)
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS())
        ;
    // set HCLK prescaler to 2 (needed for SYSCLK > 150MHz, RM0440 §6.1.5)
    // until at least 1us after SYSCLK is switched to PLLCLK
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    // select range 1 boost mode
    LL_PWR_EnableRange1BoostMode();

    // set clock source and set PLLCLK to 170MHz
#if PLATFORM_USES_EXTERNAL_CLOCK
    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() != 1)
        ;
#if !PLATFORM_EXTERNAL_CLOCK_FREQUENCY
#error PLATFORM_EXTERNAL_CLOCK_FREQUENCY must be defined if using external clock
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 4000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 8000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 12000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 16000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 24000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_6, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 32000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 85, LL_RCC_PLLR_DIV_2);
#elif PLATFORM_EXTERNAL_CLOCK_FREQUENCY == 48000000
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_12, 85, LL_RCC_PLLR_DIV_2);
#else
#error PLATFORM_EXTERNAL_CLOCK_FREQUENCY value not supported
#endif // PLATFORM_EXTERNAL_CLOCK_FREQUENCY
#else  // PLATFORM_USES_EXTERNAL_CLOCK
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1)
        ;
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
#endif // PLATFORM_USES_EXTERNAL_CLOCK

    // enable LSI
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1)
        ;

    // enable PLLCLK
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1)
        ;

    // set SYSCLK to PLLCLK (170MHz)
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
        ;

    // update SystemCoreClock CMSIS variable
    LL_SetSystemCoreClock(PLATFORM_SYSTEM_CLOCK_FREQUENCY);

    // 1us delay after SYSCLK is switched to PLLCLK (RM0440 §6.1.5)
    SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
    SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000) + 1;
    while ((DWT->CYCCNT - start) < cycles)
        ;

    // set HCLK to 170MHz
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    // clear reset cause for debugging
    LL_RCC_ClearResetFlags();
}
