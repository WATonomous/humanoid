#pragma once

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);
void AppError_Handler(void);

#ifdef __cplusplus
}
#endif