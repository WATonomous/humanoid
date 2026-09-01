#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32g4xx.h"
#include <stdint.h>

// Defines for FreeRTOS
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ (1000)
#define configMAX_PRIORITIES (5)
#define configMINIMAL_STACK_SIZE (128)
#define configTOTAL_HEAP_SIZE (8 * 1024)
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1
// #define configCHECK_FOR_STACK_OVERFLOW  2
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (5 << 4)

#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskStartScheduler 1
#define INCLUDE_vTaskPrioritySet 1

// Map FreeRTOS port interrupt handlers to STM32 ISRs
#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler

#endif