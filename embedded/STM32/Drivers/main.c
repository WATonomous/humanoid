#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

UART_HandleTypeDef huart2;
SPI_HandleTypeDef  hspi1;

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

#define MT_CS_GPIO   GPIOA
#define MT_CS_PIN    GPIO_PIN_0
#define MT_CS_LOW()  HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_RESET)
#define MT_CS_HIGH() HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_SET)

static uint32_t MT6835_ReadRaw(void)
{
    uint8_t txrx[6] = { 0xA0, 0x03, 0, 0, 0, 0 };

    MT_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, txrx, txrx, 6, HAL_MAX_DELAY);
    MT_CS_HIGH();

    uint8_t status = txrx[4] & 0x07;
    if (status) return 0;

    uint32_t raw =
        ((uint32_t)txrx[2] << 13) |
        ((uint32_t)txrx[3] <<  5) |
        (txrx[4] >> 3);

    return raw & 0x1FFFFF;
}

static float MT6835_ReadDeg(void)
{
    return MT6835_ReadRaw() * (360.0f / 2097152.0f);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    MT_CS_HIGH();

    while (1) {
        uint32_t raw  = MT6835_ReadRaw();
        uint32_t mdeg = (uint64_t)raw * 360000ULL / 2097152ULL;
        printf("Angle = %lu.%03lu deg\r\n", mdeg / 1000, mdeg % 1000);

        uint8_t burst[6] = { 0xA0, 0x03, 0, 0, 0, 0 };
        MT_CS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, burst, burst, 6, HAL_MAX_DELAY);
        MT_CS_HIGH();

        uint8_t status = burst[4] & 0x07;
        printf("RX echo: %02X %02X  status=%02X  data=%02X %02X %02X\r\n",
               burst[0], burst[1], status, burst[2], burst[3], burst[4]);

        printf("MODER PA6 = %lu\r\n", (GPIOA->MODER >> 12) & 0x3);
        HAL_Delay(500);
        HAL_Delay(50);
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 4;
    osc.PLL.PLLN            = 85;
    osc.PLL.PLLR            = RCC_PLLR_DIV2;
    osc.PLL.PLLP            = RCC_PLLP_DIV7;
    osc.PLL.PLLQ            = RCC_PLLQ_DIV2;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = MT_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MT_CS_GPIO, &g);
}

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

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

static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}
