/* src/system_init.c  – _put all init code here_ */
#include "stm32g4xx_hal.h"

/* --- bring in the global handles declared in main.c ------------------- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart2;

/* ----------------- 170 MHz core clock from HSI ------------------------ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
    
    // RCC_OscInitTypeDef osc = {0};
    // RCC_ClkInitTypeDef clk = {0};

    // osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    // osc.HSIState            = RCC_HSI_ON;
    // osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    // osc.PLL.PLLState        = RCC_PLL_ON;
    // osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    // osc.PLL.PLLM            = 2;      /* 16 MHz / 2 = 8 MHz  */
    // osc.PLL.PLLN            = 85;     /* 8 MHz × 85 = 680 MHz */
    // osc.PLL.PLLR            = 4;      /* 680 / 4 = 170 MHz -- SYSCLK */
    // HAL_RCC_OscConfig(&osc);

    // clk.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
    //                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    // clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    // clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* 170 MHz */
    // clk.APB1CLKDivider = RCC_HCLK_DIV2;        /* 85 MHz  */
    // clk.APB2CLKDivider = RCC_HCLK_DIV2;        /* 85 MHz  */
    // HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

void SysTick_Handler(void){

    HAL_IncTick();
    
}
/* ---------------- GPIO:  PB0 → CS (idle-high) ------------------------- */
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};
    io.Pin   = GPIO_PIN_0;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &io);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/* ---------------- SPI1: 16-bit, MODE-3, ≈5 MHz ------------------------ */
void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;   /* MODE-3 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; /* 170/32 ≈5 MHz */
    HAL_SPI_Init(&hspi1);
}

/* ---------------- USART2: ST-Link VCP, 9600-8-N-1 --------------------- */
void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 9600;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}
