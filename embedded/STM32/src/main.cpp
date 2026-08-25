// =====================================================================
// IMU + EKF comparison: streams raw accel/gyro/mag into our own EKF
// and prints its roll/pitch/yaw side-by-side with the BNO085's own
// built-in sensor fusion output.
//
// All output goes through UART_Print() (LPUART1), which is the
// channel wired to this board's USB port via the ST-Link's Virtual
// COM Port. Regular Arduino Serial.print() is NOT used here on
// purpose — on this board it goes to the native USB peripheral,
// which isn't physically wired to anything.
// =====================================================================

#include "Drivers/UART_STM32.h"
#include "Drivers/I2C_STM32.h"
#include "Drivers/SysClock.h"
#include "Drivers/BNO085.h"
#include "Drivers/sh2_hal_stm32.h"
#include "EKF/ekf_ahrs.h"

#include <cstdio>
#include <Arduino.h> // for HAL_Init/delay via the framework

BNO085 imu;

ekf_ahrs_t ekf;
bool ekf_initialized = false;
uint32_t last_predict_ms = 0;

void AppError_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

// ---------------------------------------------------------------------
// Bare-metal LED heartbeat: proves the chip is executing code at all,
// independent of UART/VCP/baud rate. Uses only GPIOA clock enable +
// direct register writes, so it works even before SystemClock_Config()
// runs and even if that hangs.
// ---------------------------------------------------------------------
static void heartbeat_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIOA->MODER &= ~(0x3u << (5 * 2));
  GPIOA->MODER |=  (0x1u << (5 * 2)); // PA5 as output (LD2)
}

static void heartbeat_toggle(void) {
  GPIOA->ODR ^= (1u << 5);
}

void setup() {
  heartbeat_init();
  for (int i = 0; i < 10; i++) {
    heartbeat_toggle();
    HAL_Delay(100);
  }

  HAL_Init();
  SystemClock_Config();

  MX_LPUART1_Init();
  MX_I2C1_Init();

  UART_Print("\r\n\r\n=== IMU EKF comparison starting ===\r\n");

  UART_Print("Opening SH2 driver (imu.begin())...\r\n");
  if (!imu.begin()) {
    UART_Print("=== BNO085 FAILED TO OPEN ===\r\n");
    while (1) {
      delay(1000);
    }
  }
  UART_Print("BNO085 opened successfully.\r\n");

  UART_Print("Enabling raw accelerometer/gyroscope/magnetometer reports...\r\n");
  bool accelOk = imu.enableAccelerometer();
  bool gyroOk = imu.enableGyroscope();
  bool magOk = imu.enableMagnetometer();
  {
    char msg[96];
    snprintf(msg, sizeof(msg), "accel=%s gyro=%s mag=%s\r\n",
             accelOk ? "OK" : "FAILED", gyroOk ? "OK" : "FAILED", magOk ? "OK" : "FAILED");
    UART_Print(msg);
  }

  // Let a few reports arrive before using them for the EKF mag reference.
  UART_Print("Collecting initial samples for EKF mag reference...\r\n");
  for (int i = 0; i < 100; i++) {
    imu.update();
    delay(30);
  }

  sh2_SensorValue_t magSample = imu.getMagnetometer();
  float mag_ref[3] = {
    magSample.un.magneticField.x,
    magSample.un.magneticField.y,
    magSample.un.magneticField.z
  };
  {
    char msg[96];
    snprintf(msg, sizeof(msg), "mag_ref sample: x=%.2f y=%.2f z=%.2f uT\r\n",
             mag_ref[0], mag_ref[1], mag_ref[2]);
    UART_Print(msg);
  }

  // NOTE: this assumes the board is held roughly level and still right
  // now, since it uses the CURRENT accel/mag readings as the reference
  // directions (see EKF/README.md "Calibrating mag_ref").
  ekf_ahrs_init(&ekf, mag_ref);
  last_predict_ms = HAL_GetTick();
  ekf_initialized = true;
  UART_Print("EKF initialized. Starting comparison stream...\r\n");
}

void loop() {
  heartbeat_toggle();

  if (ekf_initialized) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_predict_ms) / 1000.0f;
    last_predict_ms = now;
    if (dt <= 0.0f) dt = 0.001f;

    sh2_SensorValue_t accelSample = imu.getAccelerometer();
    sh2_SensorValue_t gyroSample = imu.getGyroscope();
    sh2_SensorValue_t magSample = imu.getMagnetometer();

    float gyro[3] = {
      gyroSample.un.gyroscope.x,
      gyroSample.un.gyroscope.y,
      gyroSample.un.gyroscope.z
    };
    float accel[3] = {
      accelSample.un.accelerometer.x,
      accelSample.un.accelerometer.y,
      accelSample.un.accelerometer.z
    };
    float mag[3] = {
      magSample.un.magneticField.x,
      magSample.un.magneticField.y,
      magSample.un.magneticField.z
    };

    ekf_ahrs_predict(&ekf, gyro, dt);
    ekf_ahrs_update_accel(&ekf, accel);
    ekf_ahrs_update_mag(&ekf, mag);

    float our_roll, our_pitch, our_yaw;
    ekf_ahrs_get_euler(&ekf, &our_roll, &our_pitch, &our_yaw);

    // Chip's own built-in fusion, for comparison.
    Quaternion cq = imu.getQuaternion();
    quat_t chip_q;
    chip_q.w = cq.w; chip_q.x = cq.x; chip_q.y = cq.y; chip_q.z = cq.z;

    // Reset our EKF's quaternion to the chip's each loop, so "OUR EKF"
    // is really "one predict+update step starting from the chip's
    // previous fusion output" rather than a freely-drifting standalone
    // filter.
    ekf.q.w = cq.w;
    ekf.q.x = cq.x;
    ekf.q.y = cq.y;
    ekf.q.z = cq.z;
    float chip_roll, chip_pitch, chip_yaw;
    quat_to_euler(chip_q, &chip_roll, &chip_pitch, &chip_yaw);

    char msg[256];
    snprintf(msg, sizeof(msg),
            "OUR EKF  roll=%.1f pitch=%.1f yaw=%.1f  |  CHIP  roll=%.1f pitch=%.1f yaw=%.1f\n",
            our_roll * 180.0f / 3.14159265f,
            our_pitch * 180.0f / 3.14159265f,
            our_yaw * 180.0f / 3.14159265f,
            chip_roll * 180.0f / 3.14159265f,
            chip_pitch * 180.0f / 3.14159265f,
            chip_yaw * 180.0f / 3.14159265f);
    UART_Print(msg);

    imu.update();
  }

  delay(5);
}