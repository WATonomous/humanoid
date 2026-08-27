#include <stm32g4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);
void MX_TIM4_Init(void);
void AppError_Handler(void);
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif