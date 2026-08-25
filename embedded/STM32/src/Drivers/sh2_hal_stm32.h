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

/**
 * Diagnostic only: returns true if the BNO08x INT line is currently
 * asserted (active low - true means the sensor claims to have data
 * ready to read).
 */
bool SH2_HAL_INT_IsAsserted(void);

/**
 * Diagnostic only: I2C-level read transaction counters.
 */
uint32_t SH2_HAL_ReadAttempts(void);
uint32_t SH2_HAL_ReadHeaderFailures(void);
uint32_t SH2_HAL_ReadPayloadFailures(void);
uint32_t SH2_HAL_ReadZeroLength(void);
uint32_t SH2_HAL_LastHeaderFailMs(void);
uint32_t SH2_HAL_WriteAttempts(void);
uint32_t SH2_HAL_WriteFailures(void);
uint32_t SH2_HAL_LastWriteFailMs(void);

#ifdef __cplusplus
}
#endif

#endif