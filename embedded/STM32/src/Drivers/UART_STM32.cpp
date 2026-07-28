#include "UART_STM32.h"
#include <cstring>

UART_HandleTypeDef hlpuart1;

void MX_LPUART1_Init(void) {
  hlpuart1.Instance = LPUART1;

  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
    while (1) {
    }
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  if (huart->Instance == LPUART1) {
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void UART_Print(const char* message) {
  if (message == nullptr) {
    return;
  }

  HAL_UART_Transmit(&hlpuart1, reinterpret_cast<uint8_t*>(const_cast<char*>(message)),
                    strlen(message), HAL_MAX_DELAY);
}