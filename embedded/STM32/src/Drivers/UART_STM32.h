#ifndef UART_STM32_H
#define UART_STM32_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef hlpuart1;

/**
 * @brief Initialize LPUART1 for communication through the Nucleo
 *        ST-LINK Virtual COM Port.
 *
 * Baud rate: 115200
 * Format: 8-N-1
 */
void MX_LPUART1_Init(void);

/**
 * @brief Transmit a null-terminated string over LPUART1.
 *
 * @param message String to transmit.
 */
void UART_Print(const char* message);

#ifdef __cplusplus
}
#endif

#endif // UART_STM32_H