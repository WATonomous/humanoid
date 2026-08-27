#include "I2C_STM32.h"

I2C_HandleTypeDef hi2c1;

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (hi2c->Instance == I2C1) {
    /* Select HSI16 as I2C1 clock source - a fixed 16MHz clock,
     * independent of SYSCLK/PCLK1, so I2C timing never needs to be
     * recalculated if the main system clock configuration changes. */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB8 (SCL) and PB9 (SDA) */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Enable I2C peripheral clock */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;

  // Computed for HSI16 (16 MHz) kernel clock, Standard Mode ~100kHz,
  // verified against RM0440 I2C_TIMINGR formulas: PRESC=3, SCLDEL=4,
  // SDADEL=2, SCLH=19, SCLL=19 -> t_SCLL=5.0us, t_SCLH=5.0us
  // (both comfortably above the I2C spec Standard Mode minimums).
  hi2c1.Init.Timing = 0x30421313;

  hi2c1.Init.OwnAddress1 = 0;

  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

  hi2c1.Init.OwnAddress2 = 0;

  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;

  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    AppError_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    AppError_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    AppError_Handler();
  }
}