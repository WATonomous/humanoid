/* mt6835.c ------------------------------------------------------------ */
#include "mt6835.h"
#include <math.h>
#include <string.h>    /* for memset if needed */
#define TWO_PI_F 6.28318530717958647692f

/* -------- private static vars -------------------------------------- */
static SPI_HandleTypeDef *hspi_;
static GPIO_TypeDef      *cs_port_;
static uint16_t           cs_pin_;

static uint16_t raw_prev = 0;
static uint32_t t_prev   = 0;
static float    angle    = 0.0f;
static float    vel      = 0.0f;

/* -------- helper ---------------------------------------------------- */
static inline void cs_low (void) { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET  ); }

/* -------- 8-bit, 2-byte read of 0xFFFF dummy ------------------------ */
static uint16_t read_raw_14bit(void)
{
    uint8_t tx[2] = {0xFF, 0xFF};
    uint8_t rx[2] = {0};

    cs_low();
    HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, HAL_MAX_DELAY);
    cs_high();

    /* MT6835 returns: D14..D7 then D6..D0 + status bits */
    uint16_t dat = ((uint16_t)rx[0] << 8) | rx[1];
    return dat & 0x3FFF;          /* 14-bit position */
}
static uint32_t read_raw_21bit(void)
{
    uint8_t txrx[6] = { 0xA0, 0x03, 0, 0, 0, 0 };  // cmd + 4 dummy
    cs_low();
    HAL_SPI_TransmitReceive(hspi_, txrx, txrx, 6, HAL_MAX_DELAY);
    cs_high();

    /* status is low 3 bits of byte 4 */
    uint8_t status = txrx[4] & 0x07;
    if (status) {
        // overspeed / weak-field / undervolt –> return 0
        return 0;
    }

    uint32_t raw =
        ((uint32_t)txrx[2] << 13) |
        ((uint32_t)txrx[3] <<  5) |
        (txrx[4] >> 3);          // 21-bit value

    return raw & 0x1FFFFF;       // mask to 21 bits
}
/* -------- public API ------------------------------------------------ */
void MT6835_Init(SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef      *cs_port,
                 uint16_t           cs_pin)
{
    hspi_    = hspi;
    cs_port_ = cs_port;
    cs_pin_  = cs_pin;

    /* idle high */
    cs_high();
}

void MT6835_Update(void)
{
    uint32_t t_now = HAL_GetTick();            /* ms */
    uint16_t raw   = read_raw_14bit();         /* 0-16383 */

    angle = (raw * TWO_PI_F) / MT6835_CPR_14BIT;

    if (t_prev != 0) {
        int16_t diff = (int16_t)(raw - raw_prev);
        /* unwrap across 0/16384 boundary */
        if (diff >  8191) diff -= 16384;
        if (diff < -8191) diff += 16384;

        float dt = (t_now - t_prev) / 1000.0f;
        vel  = (diff * TWO_PI_F) / (MT6835_CPR_14BIT * dt);
    }
    raw_prev = raw;
    t_prev   = t_now;
}

float MT6835_GetAngleRad   (void) { return angle; }
float MT6835_GetVelocityRad(void) { return vel;   }
