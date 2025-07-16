
#include "../inc/common_defines.h"
#include "../inc/system.h"
#include "stm32g4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define BOOTLOADER_SIZE (0x08008000U)

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
        uint8_t rx_buf[32] = {0};
        HAL_StatusTypeDef status;

        // wait until data is received (blocking call)
        status = HAL_UART_Receive(&huart1, rx_buf, sizeof(rx_buf) - 1, HAL_MAX_DELAY);
        if (status == HAL_OK)
        {
            // echo back
            HAL_UART_Transmit(&huart1, rx_buf, strlen((char *)rx_buf), HAL_MAX_DELAY);
        }
    }
};

int main()
{
    vector_setup();
    HAL_Init();
    system_setup();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO PIN5;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // create blinking led task
    xTaskCreate(blink_led, "BLINK_LED", 128, NULL, 1, NULL);

    // start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
        ;

    static void MX_USART1_UART_Init(void)
    {

        /* USER CODE BEGIN USART1_Init 0 */

        /* USER CODE END USART1_Init 0 */

        /* USER CODE BEGIN USART1_Init 1 */

        /* USER CODE END USART1_Init 1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
        {
            Error_Handler();
        }
        /* USER CODE BEGIN USART1_Init 2 */

        /* USER CODE END USART1_Init 2 */
    }
}