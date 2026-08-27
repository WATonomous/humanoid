#include "sh2_hal_stm32.h"
#include <cstdio>
#include "I2C_STM32.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

#include <cstring>

///////////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////////

#define BNO085_I2C_ADDR (0x4A << 1)
#define BNO085_I2C_TIMEOUT_MS 50

#define BNO085_RST_PORT GPIOA
#define BNO085_RST_PIN  GPIO_PIN_10
#define BNO085_INT_PORT GPIOA
#define BNO085_INT_PIN  GPIO_PIN_9

// Diagnostic counters, not part of normal operation
static volatile uint32_t g_readAttempts = 0;
static volatile uint32_t g_readHeaderFail = 0;
static volatile uint32_t g_readPayloadFail = 0;
static volatile uint32_t g_readZeroLength = 0;
static volatile uint32_t g_lastHeaderFailMs = 0;
static volatile uint32_t g_writeAttempts = 0;
static volatile uint32_t g_writeFailures = 0;
static volatile uint32_t g_lastWriteFailMs = 0;

///////////////////////////////////////////////////////////////////////////////
// Forward declarations
///////////////////////////////////////////////////////////////////////////////

static int SH2_Open(sh2_Hal_t* self);
static void SH2_Close(sh2_Hal_t* self);

static int SH2_Read(sh2_Hal_t* self, uint8_t* buffer, unsigned len, uint32_t* t_us);

static int SH2_Write(sh2_Hal_t* self, uint8_t* buffer, unsigned len);

static uint32_t SH2_GetTimeUs(sh2_Hal_t* self);

static void SH2_GPIO_Init(void);
static void SH2_HardwareReset(void);

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
  SH2_GPIO_Init();
  SH2_HardwareReset();

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// RST / INT pin setup
///////////////////////////////////////////////////////////////////////////////

static void SH2_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* RST: push-pull output, idle HIGH (chip runs normally when RST is high) */
  GPIO_InitStruct.Pin = BNO085_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO085_RST_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);

  /* INT: input, chip pulls this LOW when a report is ready to read */
  GPIO_InitStruct.Pin = BNO085_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO085_INT_PORT, &GPIO_InitStruct);
}

static void SH2_HardwareReset(void) {
  HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(150); /* give the sensor time to boot before talking to it */
}

///////////////////////////////////////////////////////////////////////////////

static void SH2_Close(sh2_Hal_t* self) {
  (void)self;

  // Nothing to do for now.
}

///////////////////////////////////////////////////////////////////////////////

static int SH2_Write(sh2_Hal_t* self, uint8_t* buffer, unsigned len) {
  (void)self;

  g_writeAttempts++;
  uint32_t t0 = HAL_GetTick();

  HAL_StatusTypeDef status =
      HAL_I2C_Master_Transmit(&hi2c1, BNO085_I2C_ADDR, buffer, len, BNO085_I2C_TIMEOUT_MS);

  if (status != HAL_OK) {
    g_writeFailures++;
    g_lastWriteFailMs = HAL_GetTick() - t0;
    return -1;
  }

  return (int)len;
}

///////////////////////////////////////////////////////////////////////////////

static int SH2_Read(sh2_Hal_t* self,
  uint8_t* buffer,
  unsigned len,
  uint32_t* t_us)
{
  (void)self;

  if (HAL_GPIO_ReadPin(BNO085_INT_PORT,
        BNO085_INT_PIN) == GPIO_PIN_SET)
  {
  return 0;
  }

  g_readAttempts++;

  if (HAL_I2C_Master_Receive(&hi2c1,
              BNO085_I2C_ADDR,
              buffer,
              len,
              BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
  g_readHeaderFail++;
  return -1;
  }

  uint16_t packetLength =
  (buffer[0] | ((buffer[1] & 0x7F) << 8));

  if (packetLength == 0)
  {
  g_readZeroLength++;
  return 0;
  }

  if (packetLength > len)
  {
  packetLength = len;
  }

  if (t_us)
  {
   *t_us = HAL_GetTick() * 1000UL;
  }

  return packetLength;
}

///////////////////////////////////////////////////////////////////////////////

static uint32_t SH2_GetTimeUs(sh2_Hal_t* self) {
  (void)self;

  return HAL_GetTick() * 1000UL;
}

///////////////////////////////////////////////////////////////////////////////

bool SH2_HAL_INT_IsAsserted(void) {
  return HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN) == GPIO_PIN_RESET;
}

uint32_t SH2_HAL_ReadAttempts(void) { return g_readAttempts; }
uint32_t SH2_HAL_ReadHeaderFailures(void) { return g_readHeaderFail; }
uint32_t SH2_HAL_ReadPayloadFailures(void) { return g_readPayloadFail; }
uint32_t SH2_HAL_ReadZeroLength(void) { return g_readZeroLength; }
uint32_t SH2_HAL_LastHeaderFailMs(void) { return g_lastHeaderFailMs; }
uint32_t SH2_HAL_WriteAttempts(void) { return g_writeAttempts; }
uint32_t SH2_HAL_WriteFailures(void) { return g_writeFailures; }
uint32_t SH2_HAL_LastWriteFailMs(void) { return g_lastWriteFailMs; }