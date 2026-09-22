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
static volatile uint32_t g_busRecoveries = 0;

// Sticky flag: set whenever an I2C transfer fails, cleared by whoever reads it.
static volatile bool g_transferError = false;

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

/* Clear a wedged I2C master.
 *
 * A failed transfer can leave the peripheral's state machine part-way through
 * a transaction, after which every subsequent transfer fails too. Cycling it
 * through DeInit/Init resets that state machine. (A *slave* still holding SDA
 * low is a separate problem, cleared by the hub reset in SH2_Open.) */
static void SH2_I2C_Recover(void) {
  g_busRecoveries++;

  HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
}

static int SH2_Open(sh2_Hal_t* self) {
  (void)self;

  /* Force a clean peripheral state: this path is also used for recovery after
   * the hub has stopped responding, where hi2c1 may be mid-transaction. */
  HAL_I2C_DeInit(&hi2c1);

  MX_I2C1_Init();
  SH2_GPIO_Init();
  SH2_HardwareReset();

  g_transferError = false;

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
    g_transferError = true;
    SH2_I2C_Recover();
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

  /* The hub holds INT low only while a packet is waiting to be read. */
  if (HAL_GPIO_ReadPin(BNO085_INT_PORT,
        BNO085_INT_PIN) == GPIO_PIN_SET)
  {
    return 0;
  }

  g_readAttempts++;

  /* Read the 4-byte SHTP header on its own to learn the real packet length.
   *
   * Receiving the caller's whole buffer unconditionally instead (len is
   * SH2_HAL_MAX_TRANSFER_IN = 384) costs 384*9 bits / 100 kHz = ~35 ms per
   * packet on this bus. Since shtp_service() reads exactly one packet per
   * call, that capped the entire system at ~29 packets/s while three sensors
   * were producing ~250 reports/s -- the hub's FIFO stayed permanently backed
   * up and every sample reaching the filter was tens of ms stale. A gyro
   * report packet is ~20 bytes, so reading the exact length is ~15x cheaper.
   *
   * Each new I2C read transaction restarts at the beginning of the pending
   * packet, so the second receive below re-reads the header along with the
   * payload. This is the same two-step flow the Hillcrest/Adafruit reference
   * HAL uses.
   */
  uint8_t header[4];
  uint32_t t0 = HAL_GetTick();

  if (HAL_I2C_Master_Receive(&hi2c1,
              BNO085_I2C_ADDR,
              header,
              sizeof(header),
              BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    g_readHeaderFail++;
    g_lastHeaderFailMs = HAL_GetTick() - t0;
    g_transferError = true;
    SH2_I2C_Recover();

    /* Must be 0, never -1. shtp_service() does `int len = read(...); if (len)
     * rxAssemble(..., len, ...)` and rxAssemble takes a uint16_t, so a -1
     * return arrives as 65535 and makes SHTP re-parse whatever stale bytes are
     * still sitting in its input buffer as if they were a fresh packet. */
    return 0;
  }

  /* Bit 15 of the length field is the "continuation" flag, not length. */
  uint16_t packetLength =
      (uint16_t)header[0] | ((uint16_t)(header[1] & 0x7F) << 8);

  /* A header-only packet (length 4) is the hub saying it has no cargo. */
  if (packetLength <= sizeof(header))
  {
    g_readZeroLength++;
    return 0;
  }

  if (packetLength > len)
  {
    /* Won't fit. Drop it rather than truncating -- the previous code
     * clamped to len and handed SHTP a corrupt packet. */
    g_readPayloadFail++;
    return 0;
  }

  /* Anchor the timestamp before the payload transfer, so it sits as close as
   * possible to when the hub actually raised INT. sh2 adds each report's own
   * sub-millisecond delay field to this. */
  if (t_us)
  {
    *t_us = HAL_GetTick() * 1000UL;
  }

  if (HAL_I2C_Master_Receive(&hi2c1,
              BNO085_I2C_ADDR,
              buffer,
              packetLength,
              BNO085_I2C_TIMEOUT_MS) != HAL_OK)
  {
    g_readPayloadFail++;
    g_transferError = true;
    SH2_I2C_Recover();
    return 0; /* see the note on the header-read failure path */
  }

  return (int)packetLength;
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
bool SH2_HAL_TakeTransferError(void) {
  bool err = g_transferError;
  g_transferError = false;
  return err;
}

uint32_t SH2_HAL_BusRecoveries(void) { return g_busRecoveries; }
