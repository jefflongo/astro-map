#pragma once

#include <assert.h>
#include <pthread.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.  See
 * https://www.FreeRTOS.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configTICK_RATE_HZ 1000
#define configMINIMAL_STACK_SIZE PTHREAD_STACK_MIN
#define configTOTAL_HEAP_SIZE ((size_t)(65 * 1024))
#define configMAX_TASK_NAME_LEN (12)
#define configUSE_TRACE_FACILITY 1
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_MUTEXES 1
#define configCHECK_FOR_STACK_OVERFLOW 0
#define configUSE_RECURSIVE_MUTEXES 1
#define configQUEUE_REGISTRY_SIZE 20
#define configUSE_APPLICATION_TASK_TAG 1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_ALTERNATIVE_API 0
#define configUSE_QUEUE_SETS 1
#define configUSE_TASK_NOTIFICATIONS 1

/* The following 2  memory allocation schemes are possible for this demo:
 *
 * 1. Dynamic Only.
 *    #define configSUPPORT_STATIC_ALLOCATION  0
 *    #define configSUPPORT_DYNAMIC_ALLOCATION 1
 *
 * 2. Static and Dynamic.
 *    #define configSUPPORT_STATIC_ALLOCATION  1
 *    #define configSUPPORT_DYNAMIC_ALLOCATION 1
 *
 * Static only configuration is not possible for this demo as it utilizes
 * dynamic allocation.
 */
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1

#define configRECORD_STACK_HIGH_ADDRESS 1

/* Software timer related configuration options.  The maximum possible task
 * priority is configMAX_PRIORITIES - 1.  The priority of the timer task is
 * deliberately set higher to ensure it is correctly capped back to
 * configMAX_PRIORITIES - 1. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH 20
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)

#define configMAX_PRIORITIES (7)

/* This demo can use of one or more example stats formatting functions.  These
 * format the raw data provided by the uxTaskGetSystemState() function in to human
 * readable ASCII form.  See the notes in the implementation of vTaskList() within
 * FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* Enables the test whereby a stack larger than the total heap size is
 * requested. */
#define configSTACK_DEPTH_TYPE uint32_t

/* Set the following definitions to 1 to include the API function, or zero
 * to exclude the API function.  In most cases the linker will remove unused
 * functions anyway. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_uxTaskGetStackHighWaterMark2 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle 1
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_xTaskGetHandle 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xSemaphoreGetMutexHolder 1
#define INCLUDE_xTimerPendFunctionCall 1
#define INCLUDE_xTaskAbortDelay 1

#define configASSERT(x) assert(x)

#define configUSE_MALLOC_FAILED_HOOK 0
