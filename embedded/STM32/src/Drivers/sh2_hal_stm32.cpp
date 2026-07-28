#include "sh2_hal_stm32.h"

#include "I2C_STM32.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

#include <cstring>

///////////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////////

#define BNO085_I2C_ADDR (0x4A << 1)

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////

static int SH2_Open(sh2_Hal_t* self);
static void SH2_Close(sh2_Hal_t* self);

static int SH2_Read(sh2_Hal_t* self, uint8_t* buffer, unsigned len, uint32_t* t_us);

static int SH2_Write(sh2_Hal_t* self, uint8_t* buffer, unsigned len);

static uint32_t SH2_GetTimeUs(sh2_Hal_t* self);

///////////////////////////////////////////////////////////////////////////////
// Static HAL object
///////////////////////////////////////////////////////////////////////////////

static sh2_Hal_t sh2Hal = {SH2_Open, SH2_Close, SH2_Read, SH2_Write, SH2_GetTimeUs};

///////////////////////////////////////////////////////////////////////////////

sh2_Hal_t* SH2_HAL_GetInstance(void) {
  return &sh2Hal;
}

///////////////////////////////////////////////////////////////////////////////

static int SH2_Open(sh2_Hal_t* self) {
  (void)self;

  MX_I2C1_Init();

  HAL_Delay(10);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////

static void SH2_Close(sh2_Hal_t* self) {
  (void)self;

  // Nothing to do for now.
}

///////////////////////////////////////////////////////////////////////////////

static int SH2_Write(sh2_Hal_t* self, uint8_t* buffer, unsigned len) {
  (void)self;

  HAL_StatusTypeDef status =
      HAL_I2C_Master_Transmit(&hi2c1, BNO085_I2C_ADDR, buffer, len, HAL_MAX_DELAY);

  if (status != HAL_OK)
    return -1;

  return (int)len;
}

///////////////////////////////////////////////////////////////////////////////

static int SH2_Read(sh2_Hal_t* self, uint8_t* buffer, unsigned len, uint32_t* t_us) {
  (void)self;

  uint8_t header[4];

  //-------------------------------------------------------------
  // Read SHTP header
  //-------------------------------------------------------------

  if (HAL_I2C_Master_Receive(&hi2c1, BNO085_I2C_ADDR, header, 4, HAL_MAX_DELAY) != HAL_OK) {
    return -1;
  }

  uint16_t packetLength = (header[0] | ((header[1] & 0x7F) << 8));

  if (packetLength == 0)
    return 0;

  if (packetLength > len)
    packetLength = len;

  memcpy(buffer, header, 4);

  //-------------------------------------------------------------
  // Read payload
  //-------------------------------------------------------------

  if (packetLength > 4) {
    if (HAL_I2C_Master_Receive(&hi2c1, BNO085_I2C_ADDR, buffer + 4, packetLength - 4,
                               HAL_MAX_DELAY) != HAL_OK) {
      return -1;
    }
  }

  if (t_us) {
    *t_us = HAL_GetTick() * 1000UL;
  }

  return packetLength;
}

///////////////////////////////////////////////////////////////////////////////

static uint32_t SH2_GetTimeUs(sh2_Hal_t* self) {
  (void)self;

  return HAL_GetTick() * 1000UL;
}