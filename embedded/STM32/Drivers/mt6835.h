#ifndef MT6835_H
#define MT6835_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                         */
/* ------------------------------------------------------------------ */
#define MT6835_CPR_14BIT  16384.0f   /* 2^14 */
#define PI_F              3.14159265358979323846f
#define TWO_PI_F          6.28318530717958647692f

/* ------------------------------------------------------------------ */
/*  API                                                               */
/* ------------------------------------------------------------------ */
void  MT6835_Init(SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef      *cs_port,
                  uint16_t           cs_pin);

void  MT6835_Update(void);          /* call each cycle */
float MT6835_GetAngleRad(void);     /* latest angle    */
float MT6835_GetVelocityRad(void);  /* latest velocity */

#endif /* MT6835_H */
