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
        MT6835_Update();                       /* reads and stores angle */
        float ang  = MT6835_GetAngleRad();
        float vel  = MT6835_GetVelocityRad();  /* 1st-order diff */

        /* basic printf over UART -- see retarget.c snippet below */
        printf("Angle [rad]: %.6f   Velocity [rad/s]: %.4f\r\n", ang, vel);

        /* 100 Hz loop */
        HAL_Delay(10);
    }
}
