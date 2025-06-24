/* mt6835.h -------------------------------------------------------------- */
#ifndef MT6835_H
#define MT6835_H
#include "stm32g4xx_hal.h"

void MT6835_Init(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

void MT6835_Update(void);
float MT6835_GetAngleRad(void);
float MT6835_GetVelocityRad(void);

#endif /* MT6835_H */
