#ifndef UART_STM32_H
#define UART_STM32_H

#include "stm32g4xx_hal.h"

#ifdef cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart2;

void MX_LPUART1_Init(void);
void UART_Print(const char *message);

#ifdef cplusplus
}
#endif

#endif