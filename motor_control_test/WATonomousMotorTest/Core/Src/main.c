/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : GL30 KV290 BLDC open-loop spin test
  *                   PWM: PA8/PA9/PA10 (TIM1 CH1/2/3)
  *                   Current sense: PA0/PA1 (ADC1 CH1/2)
  *                   Encoder: SPI2 PB13/PB14/PB15, CS=PB12
  *                   CAN: PA11/PA12 (FDCAN1)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <math.h>
#include <string.h>

/* USER CODE BEGIN PD */
#define PWM_ARR          4249U       // ARR value = 20kHz center-aligned at 170MHz
#define PI               3.14159265f
#define SINE_STEPS       1000        // sine table resolution
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static float    sine_table[SINE_STEPS];
static float    g_angle        = 0.0f;
static float    g_speed        = 0.001f;   // rad per interrupt ~3 rev/s, tune this
static float    g_amplitude    = 0.25f;    // 0.0 to 1.0, start low
static uint8_t  g_motor_on     = 0;        // safety flag, set to 1 to enable

// ADC DMA buffer — index 0 = phase A current, index 1 = phase B current
volatile uint16_t adc_buf[2] = {0, 0};
/* USER CODE END PV */

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
FDCAN_HandleTypeDef hfdcan1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN 0 */

// ---------------------------------------------------------------------------
// Build sine lookup table — call once at startup
// ---------------------------------------------------------------------------
static void build_sine_table(void)
{
    for (int i = 0; i < SINE_STEPS; i++)
    {
        sine_table[i] = sinf(2.0f * PI * (float)i / (float)SINE_STEPS);
    }
}

// ---------------------------------------------------------------------------
// Look up sine value for an angle in radians (0 to 2*PI)
// ---------------------------------------------------------------------------
static inline float sine_lookup(float angle)
{
    // Wrap angle to 0..2PI
    while (angle >= 2.0f * PI) angle -= 2.0f * PI;
    while (angle <  0.0f)      angle += 2.0f * PI;

    int idx = (int)((angle / (2.0f * PI)) * (float)SINE_STEPS);
    if (idx >= SINE_STEPS) idx = SINE_STEPS - 1;
    return sine_table[idx];
}

// ---------------------------------------------------------------------------
// Write 3-phase sinusoidal PWM
// angle    : electrical angle in radians
// amplitude: 0.0 = off, 1.0 = full scale
// ---------------------------------------------------------------------------
static void set_3phase_pwm(float angle, float amplitude)
{
    float a = amplitude * sine_lookup(angle);
    float b = amplitude * sine_lookup(angle + 2.0f * PI / 3.0f);
    float c = amplitude * sine_lookup(angle + 4.0f * PI / 3.0f);

    // Map -1..+1 to 0..ARR (center = ARR/2 = 50% duty)
    TIM1->CCR1 = (uint32_t)((a + 1.0f) * 0.5f * (float)PWM_ARR);
    TIM1->CCR2 = (uint32_t)((b + 1.0f) * 0.5f * (float)PWM_ARR);
    TIM1->CCR3 = (uint32_t)((c + 1.0f) * 0.5f * (float)PWM_ARR);
}

// ---------------------------------------------------------------------------
// TIM1 update interrupt — fires at 20kHz
// This is the motor control loop
// ---------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        if (g_motor_on)
        {
            g_angle += g_speed;
            if (g_angle >= 2.0f * PI) g_angle -= 2.0f * PI;
            set_3phase_pwm(g_angle, g_amplitude);
        }
        else
        {
            // Motor off — set all phases to 50% (zero voltage vector)
            TIM1->CCR1 = PWM_ARR / 2;
            TIM1->CCR2 = PWM_ARR / 2;
            TIM1->CCR3 = PWM_ARR / 2;
        }
    }
}

// ---------------------------------------------------------------------------
// MT6835 encoder — read raw 21-bit angle over SPI2
// Returns angle as 0..2097151 (2^21 - 1)
// ---------------------------------------------------------------------------
uint32_t MT6835_ReadAngle(void)
{
    uint8_t tx[4] = {0x83, 0x00, 0x00, 0x00}; // Read angle register 0x003
    uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CS low
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // CS high

    // Angle is bits [23:3] of the response = 21 bits
    uint32_t raw = ((uint32_t)rx[1] << 13) |
                   ((uint32_t)rx[2] << 5)  |
                   ((uint32_t)rx[3] >> 3);
    return raw & 0x1FFFFF;
}

// ---------------------------------------------------------------------------
// Convert raw MT6835 reading to radians 0..2PI
// ---------------------------------------------------------------------------
float MT6835_ReadAngleRad(void)
{
    uint32_t raw = MT6835_ReadAngle();
    return ((float)raw / 2097152.0f) * 2.0f * PI;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_FDCAN1_Init();
    MX_SPI2_Init();

    /* USER CODE BEGIN 2 */

    // Build sine table
    build_sine_table();

    // CS pin idle high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    // Fix TIM1 TRGO to trigger ADC at update event
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // Start ADC with DMA — fills adc_buf[0] and adc_buf[1] automatically
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);

    // Start PWM on all 3 channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Start TIM1 update interrupt (runs the motor control loop at 20kHz)
    HAL_TIM_Base_Start_IT(&htim1);

    // -----------------------------------------------------------------------
    // SAFETY: motor is OFF by default
    // Set g_motor_on = 1 to start spinning
    // Start with low amplitude (0.15 to 0.25) and increase slowly
    // If motor gets hot or vibrates badly, stop immediately
    // -----------------------------------------------------------------------
    HAL_Delay(500);       // let everything settle
    g_amplitude = 0.20f;  // 20% — safe starting point for GL30
    g_speed     = 0.001f; // slow speed
    g_motor_on  = 1;      // ENABLE MOTOR — comment this out for safe testing

    /* USER CODE END 2 */

    BSP_LED_Init(LED_GREEN);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    while (1)
    {
        /* USER CODE BEGIN 3 */

        // Blink LED to show code is running
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(500);

        // Optional debug — read encoder angle
        // float enc_angle = MT6835_ReadAngleRad();
        // (set a breakpoint here and watch enc_angle in the debugger)

        /* USER CODE END 3 */
    }
}

// ============================================================================
// All generated init functions below — do not modify
// ============================================================================

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = DISABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 17;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 5;
    hfdcan1.Init.NominalTimeSeg2      = 4;
    hfdcan1.Init.DataPrescaler        = 1;
    hfdcan1.Init.DataSyncJumpWidth    = 1;
    hfdcan1.Init.DataTimeSeg1         = 1;
    hfdcan1.Init.DataTimeSeg2         = 1;
    hfdcan1.Init.StdFiltersNbr        = 0;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
}

static void MX_SPI2_Init(void)
{
    hspi2.Instance              = SPI2;
    hspi2.Init.Mode             = SPI_MODE_MASTER;
    hspi2.Init.Direction        = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize         = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity      = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase         = SPI_PHASE_1EDGE;
    hspi2.Init.NSS              = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit         = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode           = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation   = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial    = 7;
    hspi2.Init.CRCLength        = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode         = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig     = {0};
    TIM_MasterConfigTypeDef sMasterConfig         = {0};
    TIM_OC_InitTypeDef sConfigOC                  = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = 4249;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;  // triggers ADC
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB12 = SPI2 CS, start high (deselected)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
