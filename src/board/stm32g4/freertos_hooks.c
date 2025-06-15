#include <FreeRTOS.h>
#include <task.h>

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    portDISABLE_INTERRUPTS();
    configASSERT(0);
}

void vApplicationMallocFailedHook(void) {
    portDISABLE_INTERRUPTS();
    configASSERT(0);
}
