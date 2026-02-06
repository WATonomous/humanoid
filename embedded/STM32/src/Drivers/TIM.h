#include <stm32g4xx_hal_tim.h>

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);
void MX_TIM4_Init(void);
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif