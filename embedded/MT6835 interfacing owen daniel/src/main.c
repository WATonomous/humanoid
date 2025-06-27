#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ─────────────  Forward decls ─────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* ─────────────  HAL handles ───────────────────────────── */
UART_HandleTypeDef huart2;
SPI_HandleTypeDef  hspi1;

/* ─────────────  Retarget printf() to USART2 ───────────── */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ─────────────  MT6835 helpers ────────────────────────── */
#define MT_CS_GPIO   GPIOB
#define MT_CS_PIN    GPIO_PIN_0
#define MT_CS_LOW()  HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_RESET)
#define MT_CS_HIGH() HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_SET)

/* read 21-bit raw angle (0…2 097 151) */
static uint32_t MT6835_ReadRaw(void)
{
    uint8_t cmd[3] = {0xA0, 0x03, 0x00};   /* burst read @0x003 */
    uint8_t angle[4] = {0};

    MT_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, sizeof cmd, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, angle, 4, HAL_MAX_DELAY);
    MT_CS_HIGH();

    uint32_t raw = ((uint32_t)angle[0] << 13) |
                   ((uint32_t)angle[1] <<  5) |
                   ((uint32_t)angle[2] >>  3);
    return raw & 0x1FFFFF;
}

static float MT6835_ReadDeg(void)
{
    return MT6835_ReadRaw() * (360.0f / 2097152.0f);   /* 2^21 */
}

/* ─────────────  main() ───────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    MT_CS_HIGH();                       /* idle high */

    while (1) {
        printf("Angle: %.2f deg\r\n", MT6835_ReadDeg());
        HAL_Delay(50);
    }
}

/* ─────────────  Peripheral init code ───────────────────
   Minimal, self-contained, no CubeMX required            */

static void SystemClock_Config(void)
{
    /* 170 MHz HSI / PLL (ST app-note AN5419) -- trimmed */
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

    /* PB0  -> CS */
    GPIO_InitTypeDef g = {0};
    g.Pin   = MT_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MT_CS_GPIO, &g);

    /* PA2/PA3 already configured by UART init */
    /* PA5/6/7 configured by SPI init         */
}

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 = TX, PA3 = RX */
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

    /* PA5 = SCK, PA6 = MISO, PA7 = MOSI  (AF5) */
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
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;   /* mode-3 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; /* ≈5 MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}
