#ifndef SH2_HAL_STM32_H
#define SH2_HAL_STM32_H

#include "sh2_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns the SH-2 HAL instance used by the STM32 implementation.
 */
sh2_Hal_t* SH2_HAL_GetInstance(void);

#ifdef __cplusplus
}
#endif

#endif