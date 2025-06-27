#include "stm32g4xx_hal.h"
#include "mt6835.h"
#include <stdio.h> 

/* --- Global handles ---------------------------------------------------- */
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* --- Forward declarations --------------------------------------------- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}
/* --- Application ------------------------------------------------------- */
int main(void)
{
    HAL_Init();                /* reset peripherals / Systick */
    SystemClock_Config();      /* 84 MHz internal → PLL */
    MX_GPIO_Init();            /* CS + LED */
    MX_SPI1_Init();            /* 500 kHz, MODE3 */
    MX_USART2_UART_Init();     /* 9600 Bd, 8-N-1 */

/* Replace the old line: */
    MT6835_Init(&hspi1, GPIOB, GPIO_PIN_0);   // PB0 → CS

    uint32_t t_prev = HAL_GetTick();

    while (1)
    {
        

    MT6835_Update();  // reads and stores angle

    // Read the values
    float ang = MT6835_GetAngleRad();
    float vel = MT6835_GetVelocityRad();

    // Format into buffer
    char buf[64];
    int  len = snprintf(buf, sizeof(buf),
                        "Angle [rad]: %.6f   Velocity [rad/s]: %.4f\r\n",
                        ang, vel);

    // Transmit over UART2
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)buf,
                      len,
                      HAL_MAX_DELAY);

    // 100 Hz loop
    HAL_Delay(10);
}
    }

