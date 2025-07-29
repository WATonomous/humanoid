
#include "../inc/common_defines.h"
#include "../inc/system.h"
#include "stm32g4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define BOOTLOADER_SIZE (0x08008000U)
UART_HandleTypeDef huart1;

// set the vector table offset of app firmware image
static void vector_setup(void)
{
    SCB->VTOR = BOOTLOADER_SIZE;
}

// thread for blinking led
void blink_led(void *pvParams)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        // vTaskDelay(pdMS_TO_TICKS(500));
        HAL_Delay(500);
    }
}
void stm_to_esp()
{
    while (1)
    {
        char buffer[16] = {0};
        char rx_buf[2] = {0};
        sprintf(buffer, "Test: \n");
        //HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
        //HAL_Delay(500);

        // wait until data is received (blocking call)
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, (uint8_t*)rx_buf, 2, HAL_MAX_DELAY);
        if (status == HAL_OK)
        {
            // echo back
            HAL_UART_Transmit(&huart1, (uint8_t*)rx_buf, strlen(rx_buf), HAL_MAX_DELAY);
            HAL_Delay(500);
        }
    }
}

static void MX_USART2_Init(void)
{
    huart1.Instance = USART2;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}
int main()
{
    vector_setup();
    HAL_Init();
    system_setup();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    MX_USART2_Init();

    // Configure GPIO PIN5;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // create blinking led task
    xTaskCreate(blink_led, "BLINK_LED", 128, NULL, 1, NULL);

    xTaskCreate(stm_to_esp, "STM_TO_ESP", 128, NULL, 1, NULL);

    // start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
        // HAL_Delay(500);
    }
}