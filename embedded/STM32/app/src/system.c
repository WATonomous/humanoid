#include "../inc/system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"
// #include "stm32g4xx_hal_pwr_ex.c"

UART_HandleTypeDef hlpuart1;

extern void xPortSysTickHandler(void);

void MX_LPUART1_UART_Init(void) {
  hlpuart1.Instance = LPUART1; // type
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE; // corruption protection
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&hlpuart1); // calls init, including the HAL_UART_MspInit
}

// Overriding the HAL UART init for the usb UART protocol
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // if its the LPUART, then enable clocks and GPIO mode
  if (huart->Instance == LPUART1) {
    __HAL_RCC_LPUART1_CLK_ENABLE(); // enable clocks
    __HAL_RCC_GPIOA_CLK_ENABLE(); // pins also need own clock

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3; // | lets you set multiple pins at once, considered bitmaps? 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // not GPIO, setting up for UART
    GPIO_InitStruct.Pull = GPIO_NOPULL; // no pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // just preference?
    GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1; // the alternate mode other than GPIO, being the UART
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // automatically calls the GPIO pin initializer
  }
}

void SysTick_Handler(void) {
  // increment hal global counter to use hal_delay()
  HAL_IncTick();
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    xPortSysTickHandler();
  }
}

void system_setup(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Just run on HSI 16MHz, no PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  MX_LPUART1_UART_Init();
}

void echo_task( void *pvParameters ) {

  // the storage location in byte type, passed by reference to the functions in order to read/modify
  uint8_t byte;

  vTaskDelay(pdMS_TO_TICKS(10000));
  HAL_UART_Transmit( &hlpuart1, (uint8_t *)"Echo Task: Enabled\r\n", 20, HAL_MAX_DELAY );

  // infinitely check serial monitor and then echo the character
  while( true ) {
    HAL_UART_Receive( &hlpuart1, &byte, 1, HAL_MAX_DELAY );
    byte++;
    HAL_UART_Transmit( &hlpuart1, &byte, 1, HAL_MAX_DELAY );
  }
}