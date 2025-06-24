/* mt6835.c -------------------------------------------------------------- */
#include "mt6835.h"
#include <math.h>

static SPI_HandleTypeDef *hspi_;
static GPIO_TypeDef *cs_port_;
static uint16_t cs_pin_;

static uint16_t raw_prev = 0;
static uint32_t t_prev   = 0;
static float    angle    = 0.0f;
static float    vel      = 0.0f;

void MT6835_Init(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    hspi_    = hspi;
    cs_port_ = cs_port;
    cs_pin_  = cs_pin;
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

static uint16_t readRaw(void)
{
    uint16_t dout = 0xFFFF;   /* dummy */
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi_, (uint8_t *)&dout,
                                   (uint8_t *)&dout, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    return dout & 0x3FFF;     /* 14-bit position data */
}

void MT6835_Update(void)
{
    uint32_t t_now = HAL_GetTick();
    uint16_t raw   = readRaw();

    /* unwrap, convert to rad */
    angle = (raw * 2.0f * (float)M_PI) / 16384.0f;

    if (t_prev != 0) {
        int16_t diff = (int16_t)(raw - raw_prev);
        /* handle rollover (14-bit) */
        if (diff >  8191) diff -= 16384;
        if (diff < -8191) diff += 16384;

        float dt = (t_now - t_prev) / 1000.0f;      /* s */
        vel = ((float)diff * 2.0f * (float)M_PI) / (16384.0f * dt);
    }
    raw_prev = raw;
    t_prev   = t_now;
}

float MT6835_GetAngleRad(void)     { return angle; }
float MT6835_GetVelocityRad(void)  { return vel;   }
