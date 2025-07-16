#include <stm32g4xx_hal_fdcan.h>
#include <SimpleFOC.h>

#ifdef __cplusplus
extern "C" {
#endif
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);
void MX_FDCAN2_Init(void);
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle);
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle);

#ifdef __cplusplus
}
#endif
