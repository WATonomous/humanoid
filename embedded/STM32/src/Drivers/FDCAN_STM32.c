/* USER CODE BEGIN PFP */

#include "FDCAN_STM32.h"
#include <SimpleFOC.h>
FDCAN_HandleTypeDef hfdcan2;

static void check_can_bus(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_ProtocolStatusTypeDef protocolStatus = {};

  HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
  if (protocolStatus.BusOff) {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if (hfdcan == &hfdcan2) {
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
      check_can_bus(hfdcan);
    }
  }
}

void MX_FDCAN2_Init(void) {

  /* USER CODE BEGIN FDCAN2_Init 0 */

	@@ -49,7 +58,8 @@ void MX_FDCAN2_Init(void) {
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
	@@ -59,11 +69,13 @@ void MX_FDCAN2_Init(void) {

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (fdcanHandle->Instance == FDCAN2) {
    /* USER CODE BEGIN FDCAN2_MspInit 0 */

    /* USER CODE END FDCAN2_MspInit 0 */
	@@ -72,13 +84,15 @@ void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

	@@ -105,15 +119,18 @@ void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle) {
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle) {

  if (fdcanHandle->Instance == FDCAN2) {
    /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

    /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

	@@ -131,3 +148,36 @@ void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle) {
    /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}