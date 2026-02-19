/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "rtc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "spi.h"
#include "mt6835.h"
#include "tim.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef htim4;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData_C2_To_C3[64];
uint8_t RxData_C3[8];
volatile int txDone = 0;
uint32_t prevTime = 0;
uint32_t currTime = 0;
float duration = 100;
const uint8_t phases = 3;
static uint8_t stepCnt = 1;

#define MT_CS_GPIO   GPIOA
#define MT_CS_PIN    GPIO_PIN_0
#define MT_CS_LOW()  HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_RESET)
#define MT_CS_HIGH() HAL_GPIO_WritePin(MT_CS_GPIO, MT_CS_PIN, GPIO_PIN_SET)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void check_can_bus(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_ProtocolStatusTypeDef protocolStatus = {};

  HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
  if (protocolStatus.BusOff) {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan == &hfdcan2) {
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
      check_can_bus(hfdcan);
    }
  }
}

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

static void killMotors(void) {
  HAL_GPIO_WritePin(MOTOR_PORT, PHASE_A_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORT, PHASE_B_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_PORT, PHASE_C_PIN, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

static float MT6835_ReadDeg(void)
{
    return MT6835_ReadRaw() * (360.0f / 2097152.0f);
}

static void StatorPWMMove() {
  //U = A, W = C, V = B
  //CH_1 = A, CH_2 = B, CH_4 = C
   killMotors();
  switch (stepCnt) {
    case 1: // A+, B-, C float
          HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_C_PIN, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_B_PIN, GPIO_PIN_RESET); 

   
      break;
    case 2: // A+, C-, B float
          HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_B_PIN, GPIO_PIN_RESET); 

      break;
    case 3: // B+, C-, A float
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_A_PIN, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_B_PIN, GPIO_PIN_RESET); 

      break;
    case 4: // B+, A-, C float
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_C_PIN, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_A_PIN, GPIO_PIN_RESET); 
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  
      break;
    case 5: // C+, A-, B float
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
      HAL_GPIO_WritePin(MOTOR_PORT, PHASE_C_PIN, GPIO_PIN_RESET); 
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      stepCnt = 0; 
      break;
    case 6: // C+, B-, A float
      break;
  }

  stepCnt++;
}





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_USART2_UART_Init();
  MX_USB_Device_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  
  //MT6835_Init(&hspi1, MT_CS_GPIO, MT_CS_PIN);

  /* USER CODE BEGIN 2 */
  TxHeader.Identifier = 0x123; // Standard ID
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  // TxData_C2_To_C3[0]=0b10101010;
  // for(uint8_t i=1;i<64;i++){
	//   TxData_C2_To_C3[i]=2*i+2;
  // }
//
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x123;
  sFilterConfig.FilterID2 = 0x123;

  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK
		  || HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK) {
	  Error_Handler();
  }

  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 38, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);


  HAL_FDCAN_ActivateNotification(
      &hfdcan2,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |   // RX FIFO0 new message
      FDCAN_IT_TX_COMPLETE |            // TX completed and acknowledged
      FDCAN_IT_ERROR_WARNING |          // error warning limit reached
      FDCAN_IT_BUS_OFF |                // bus-off state
      FDCAN_IT_ARB_PROTOCOL_ERROR,          // protocol error detected
      0
  );
  HAL_FDCAN_ActivateNotification(
      &hfdcan3,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |   // RX FIFO0 new message
      FDCAN_IT_TX_COMPLETE |            // TX completed and acknowledged
      FDCAN_IT_ERROR_WARNING |          // error warning limit reached
      FDCAN_IT_BUS_OFF |                // bus-off state
      FDCAN_IT_ARB_PROTOCOL_ERROR,          // protocol error detected
      0
  );

  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);

  //thread
  
  //Timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    currTime = HAL_GetTick();

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData_C2_To_C3) != HAL_OK) {
        Error_Handler();
    }


    while (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, FDCAN_TX_BUFFER0)) {
        FDCAN_ProtocolStatusTypeDef status;
        HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status);
        printf("Hello from USB CDC! Tick: %lu\r\n", HAL_GetTick());
              printf("BusOff: %d, Activity: %d, LastErrorCode: %d, RX FIFO Fill: %lu\n",
          status.BusOff, status.Activity, status.LastErrorCode,
          HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0));
    };
    //MT6835_Update();

    uint32_t raw  = MT6835_ReadRaw();
    uint32_t mdeg = (uint64_t)raw * 360000ULL / 2097152ULL;
    printf("Angle! = %lu.%03lu deg\r\n", mdeg / 1000, mdeg % 1000);
    TxData_C2_To_C3[0] = (mdeg >> 24) & 0xFF;
    TxData_C2_To_C3[1] = (mdeg >> 16) & 0xFF;
    TxData_C2_To_C3[2] = (mdeg >> 8) & 0xFF;
    TxData_C2_To_C3[3] = mdeg & 0xFF;
    uint8_t burst[6] = { 0xA0, 0x03, 0, 0, 0, 0 };
    MT_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, burst, burst, 6, HAL_MAX_DELAY);
    MT_CS_HIGH();
    
    TIM4->CCR4 = (RxData_C3[0]-'0') * 10 + (RxData_C3[1]-'0');  // Set the maximum pulse (2ms)
    TIM4->CCR2 = (RxData_C3[0]-'0') * 10 + (RxData_C3[1]-'0');  // Set the maximum pulse (2ms)
    TIM4->CCR1 = (RxData_C3[0]-'0') * 10 + (RxData_C3[1]-'0');  // Set the maximum pulse (2ms)
 
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Phase A PWM
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Phase A PWM
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Phase A PWM

    if (TIM4->CCR4 != 0) { //catch undefined log
      StatorPWMMove();
      HAL_Delay(duration - 
      duration * (log10(TIM4->CCR4)/log10(duration*exp(1))));
    }
    else {
      HAL_Delay(100);
    }
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	 if(hfdcan == &hfdcan2) {
	    txDone = 1;
	 }
}

int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData_C3);
    uint8_t duty = (RxData_C3[0] - '0') * 10 + (RxData_C3[1] - '0');
    printf("Duty Cycle percent: %d\r\n",duty);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
