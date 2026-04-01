
#include "../inc/common_defines.h"
#include "../inc/system.h"
#include "FreeRTOS.h"
#include "stm32g4xx.h"
#include "task.h"

#define BOOTLOADER_SIZE (0x08008000U)

// set the vector table offset of app firmware image
static void vector_setup(void) { SCB->VTOR = BOOTLOADER_SIZE; }

// thread for blinking led
void blink_led(void *pvParams) {
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // vTaskDelay(pdMS_TO_TICKS(500));
    HAL_Delay(500);
  }
}

int main() {
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

  /*
  function,
  debug string name,
  stack size (bytes * 4 since its in "words", and each word is 4 bytes),
  pvParameters passed into the task function if needed (ex. we can pass and dereference the huart),
  priority (lower number = higher),
  variable of type TaskHandle_t to use with suspending, reenabling, and stopping this specific task
  */
  xTaskCreate(echo_task, "echo_task", 128, NULL, 1, NULL);
  // create blinking led task
  xTaskCreate(blink_led, "BLINK_LED", 128, NULL, 1, NULL);

  // start FreeRTOS Scheduler
  vTaskStartScheduler();

  while (1)
    ;
}