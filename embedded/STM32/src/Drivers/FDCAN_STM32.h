#include <stm32g4xx_hal_fdcan.h>

#ifdef __cplusplus
extern "C" {
#endif
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);
void MX_FDCAN2_Init(void);
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle);
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle);
extern FDCAN_HandleTypeDef hfdcan2;


#ifdef __cplusplus
}
#endif

static void check_can_bus(FDCAN_HandleTypeDef *hfdcan);

