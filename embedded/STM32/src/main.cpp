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

// ---------------------------------------------------------------------
// Test mode.
//
// false: gyro + accel only. Our EKF never sees the magnetometer, and it is
//        compared against the hub's GAME rotation vector -- the chip's own
//        gyro + accel fusion. Same inputs on both sides, so any disagreement
//        is down to the filter itself, not magnetic distortion. Both yaws
//        drift slowly (nothing observes heading); what matters is whether
//        they drift TOGETHER.
// true:  full 9-axis. Magnetometer fused, compared against the hub's
//        mag-referenced rotation vector.
// ---------------------------------------------------------------------
static constexpr bool kFuseMagnetometer = false;

BNO085 imu;

ekf_ahrs_t ekf;
bool ekf_initialized = false;
uint32_t stream_start_ms = 0;

// The chip-side fusion that matches the inputs our EKF is using.
static Quaternion ChipQuaternion() {
  return kFuseMagnetometer ? imu.getQuaternion() : imu.getGameQuaternion();
}

// Timestamp of the last gyro report consumed, in the sensor hub's own
// microsecond timebase. dt comes from this rather than from HAL_GetTick(),
// so the integration interval matches the sample it is integrating.
uint64_t last_gyro_ts = 0;

// Gyro predicts since the last health check in loop(). Should track the
// enabled gyro rate (100 Hz); a shortfall means the transport is starving
// the filter again.
uint32_t predict_count = 0;

// Fold an angle difference in degrees into (-180, 180].
static float wrap_deg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

void AppError_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

// ---------------------------------------------------------------------
// Every decoded report steps the filter immediately, at that sensor's own
// rate and against its own timestamp.
//
// Running the filter once per control loop instead meant integrating only
// the most recent gyro sample over the whole loop period and discarding the
// rest. At a 43 ms loop against a 100 Hz gyro that threw away three of every
// four samples and stretched the survivor over 4x its true interval, so
// integrated angle error accumulated whenever the board actually moved --
// which is exactly when the disagreement with the chip appeared.
// ---------------------------------------------------------------------
static void OnImuReport(const sh2_SensorValue_t& v, void* ctx) {
  (void)ctx;

  if (!ekf_initialized) return;

  switch (v.sensorId) {
  case SH2_GYROSCOPE_CALIBRATED: {
    if (last_gyro_ts != 0) {
      float dt = (float)(v.timestamp - last_gyro_ts) * 1e-6f;

      // Guard against a backwards or absurd interval (hub restart, a stall
      // long enough that integrating across it is meaningless). Skipping the
      // predict is far better than injecting a bogus dt.
      if (dt > 0.0f && dt < 0.2f) {
        float gyro[3] = {
          v.un.gyroscope.x,
          v.un.gyroscope.y,
          v.un.gyroscope.z
        };
        ekf_ahrs_predict(&ekf, gyro, dt);
        predict_count++;
      }
    }
    last_gyro_ts = v.timestamp;
    break;
  }

  case SH2_ACCELEROMETER: {
    float accel[3] = {
      v.un.accelerometer.x,
      v.un.accelerometer.y,
      v.un.accelerometer.z
    };
    float accel_norm = sqrtf(accel[0] * accel[0] +
                             accel[1] * accel[1] +
                             accel[2] * accel[2]);

    // Only trust the accelerometer as a gravity reference when it looks
    // like mostly gravity; under linear acceleration it fights real motion.
    if (fabsf(accel_norm - 9.81f) < 1.0f) {
      ekf_ahrs_update_accel(&ekf, accel);
    }
    break;
  }

  case SH2_MAGNETIC_FIELD_CALIBRATED: {
    if (!kFuseMagnetometer) break;

    float mag[3] = {
      v.un.magneticField.x,
      v.un.magneticField.y,
      v.un.magneticField.z
    };
    ekf_ahrs_update_mag(&ekf, mag);
    break;
  }

  default:
    break;
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

// The hub's 0..3 accuracy estimate carried in every report's status byte.
static uint8_t Accuracy(const sh2_SensorValue_t& v) {
  return v.status & 0x3;
}

// ---------------------------------------------------------------------
// Interactive calibration. The BNO085 calibrates itself continuously
// (dynamic calibration) and applies the result to the *_CALIBRATED reports
// we consume -- it just needs the right motion to converge. This walks the
// user through that motion, waits until the hub reports the magnetometer
// fully calibrated, then saves the result to the hub's flash so later boots
// start calibrated.
//
// Two independent signs of a good mag calibration are printed:
//   - acc=3: the hub's own verdict.
//   - |B| spread: Earth's field has a fixed magnitude, so once hard/soft-iron
//     errors are removed |B| should barely change however the board is
//     turned. Uncalibrated, ours swung 45-60 uT (~30%).
// ---------------------------------------------------------------------
static bool RunCalibration() {
  UART_Print(
      "\r\n=== CALIBRATION ===\r\n"
      "Do this with the IMU in its FINAL mounting (rigidly fixed), away from\r\n"
      "laptops, phones and metal. Calibration is only valid for that setup.\r\n"
      "  1. Set it down, still, for ~3 s                     (gyro)\r\n"
      "  2. Hold it still in 4-6 different orientations, ~2 s each (accel)\r\n"
      "  3. Slowly rotate it in figure-8s through all axes    (mag)\r\n"
      "Done when mag acc=3 holds for 3 s. Watch |B| spread fall to a few percent.\r\n\r\n");

  const uint32_t kHoldMs = 3000;
  const uint32_t kTimeoutMs = 120000;

  uint32_t start = HAL_GetTick();
  uint32_t last_print = start;
  uint32_t good_since = 0;
  uint64_t last_mag_ts = 0;
  float b_min = 1e9f, b_max = 0.0f;

  while (true) {
    imu.update();
    uint32_t now = HAL_GetTick();

    sh2_SensorValue_t m = imu.getMagnetometer();
    if (m.timestamp != last_mag_ts) {
      last_mag_ts = m.timestamp;
      float b = sqrtf(m.un.magneticField.x * m.un.magneticField.x +
                      m.un.magneticField.y * m.un.magneticField.y +
                      m.un.magneticField.z * m.un.magneticField.z);
      if (b < b_min) b_min = b;
      if (b > b_max) b_max = b;
    }

    uint8_t acc_mag = Accuracy(m);
    if (acc_mag >= 3) {
      if (good_since == 0) good_since = now;
    } else {
      good_since = 0;
    }

    if (now - last_print >= 500) {
      last_print = now;
      float spread = (b_max > b_min) ? 200.0f * (b_max - b_min) / (b_max + b_min) : 0.0f;

      char msg[192];
      snprintf(msg, sizeof(msg),
               "CAL t=%5.1fs  acc: accel=%u gyro=%u mag=%u  |B| %.1f-%.1f uT (spread %.0f%%)"
               "  heading_acc=%.1f deg\r\n",
               (now - start) / 1000.0f,
               (unsigned)Accuracy(imu.getAccelerometer()),
               (unsigned)Accuracy(imu.getGyroscope()),
               (unsigned)acc_mag, b_min, b_max, spread,
               imu.headingAccuracy() * 180.0f / 3.14159265f);
      UART_Print(msg);

      b_min = 1e9f;
      b_max = 0.0f;
    }

    if (good_since != 0 && now - good_since >= kHoldMs) {
      bool saved = imu.saveCalibration();
      UART_Print(saved ? "Calibration converged and SAVED to BNO085 flash.\r\n"
                       : "Calibration converged, but saving to flash FAILED.\r\n");
      return true;
    }

    if (now - start >= kTimeoutMs) {
      UART_Print("Calibration TIMED OUT before mag reached acc=3. Nothing saved.\r\n");
      return false;
    }
  }
}

// ---------------------------------------------------------------------
// Heading reference capture. Waits until the board is genuinely still, then
// averages the body-frame field over ~1.5 s and grabs the hub's fused
// attitude at the same moment. The average removes sample noise, and doing it
// only while still avoids pairing a field sample with an attitude from a
// slightly different instant.
// ---------------------------------------------------------------------
static bool CaptureStillReference(float mag_avg[3], Quaternion* q_out) {
  UART_Print("\r\nCapturing heading reference: set the IMU down and keep it still...\r\n");

  const float kStillRadPerSec = 0.02f; // ~1.1 deg/s
  const uint32_t kSettleMs = 1000;
  const uint32_t kAverageMs = 1500;
  const uint32_t kTimeoutMs = 30000;

  uint32_t start = HAL_GetTick();
  uint32_t still_since = 0;
  uint64_t last_mag_ts = 0;
  float sum[3] = {0.0f, 0.0f, 0.0f};
  int n = 0;

  while (HAL_GetTick() - start < kTimeoutMs) {
    imu.update();
    uint32_t now = HAL_GetTick();

    sh2_SensorValue_t g = imu.getGyroscope();
    float w = sqrtf(g.un.gyroscope.x * g.un.gyroscope.x +
                    g.un.gyroscope.y * g.un.gyroscope.y +
                    g.un.gyroscope.z * g.un.gyroscope.z);

    // Any motion restarts the whole capture.
    if (w > kStillRadPerSec) {
      still_since = 0;
      sum[0] = sum[1] = sum[2] = 0.0f;
      n = 0;
      continue;
    }

    if (still_since == 0) still_since = now;
    if (now - still_since < kSettleMs) continue;

    sh2_SensorValue_t m = imu.getMagnetometer();
    if (m.timestamp != last_mag_ts) {
      last_mag_ts = m.timestamp;
      sum[0] += m.un.magneticField.x;
      sum[1] += m.un.magneticField.y;
      sum[2] += m.un.magneticField.z;
      n++;
    }

    if (now - still_since >= kSettleMs + kAverageMs && n >= 10) {
      mag_avg[0] = sum[0] / n;
      mag_avg[1] = sum[1] / n;
      mag_avg[2] = sum[2] / n;
      *q_out = imu.getQuaternion();
      return true;
    }
  }

  return false;
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

  // Report rates are chosen against the I2C budget: at 100 kHz each packet
  // costs ~2.3 ms of bus time. The gyro drives prediction and wants the full
  // rate; accel and mag only correct slowly-varying references. Only the one
  // chip-side fusion output we actually compare against is left enabled.
  UART_Print(kFuseMagnetometer
                 ? "MODE: 9-axis (gyro+accel+mag) vs chip ROTATION VECTOR\r\n"
                 : "MODE: 6-axis (gyro+accel, NO mag) vs chip GAME ROTATION VECTOR\r\n");

  bool gyroOk = imu.enableGyroscope(10000);      // 100 Hz
  bool accelOk = imu.enableAccelerometer(20000); //  50 Hz
  bool magOk = true;
  bool chipOk;

  if (kFuseMagnetometer) {
    magOk = imu.enableMagnetometer(50000);       //  20 Hz
    chipOk = true;                               // enabled by begin()

    // Calibrate all three sensors dynamically, but only ever write to flash
    // explicitly, after RunCalibration() has confirmed convergence. With
    // auto-save on, the hub periodically persists whatever it currently has
    // -- including a calibration corrupted by nearby metal.
    imu.setCalibrationAutoSave(false);
    imu.setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  } else {
    // begin() turns the mag-referenced rotation vector on; it is dead weight
    // on the bus in this mode.
    imu.disableRotationVector();
    chipOk = imu.enableGameRotationVector(20000); //  50 Hz
  }

  {
    char msg[128];
    snprintf(msg, sizeof(msg), "accel=%s gyro=%s mag=%s chip_fusion=%s\r\n",
             accelOk ? "OK" : "FAILED", gyroOk ? "OK" : "FAILED",
             kFuseMagnetometer ? (magOk ? "OK" : "FAILED") : "off",
             chipOk ? "OK" : "FAILED");
    UART_Print(msg);
  }

  // Let reports arrive (and the chip's fusion settle) before seeding.
  UART_Print("Collecting initial samples...\r\n");
  for (int i = 0; i < 100; i++) {
    imu.update();
    delay(30);
  }

  // Placeholder mag reference; replaced below in 9-axis mode, unused in
  // 6-axis mode.
  const float mag_ref_placeholder[3] = {0.0f, 1.0f, 0.0f};
  ekf_ahrs_init(&ekf, mag_ref_placeholder);

  // Seed our attitude from the chip fusion we compare against, so both
  // start in the same nav frame. In 6-axis mode this also fixes the shared
  // (arbitrary) yaw origin -- the game rotation vector has no absolute
  // heading. In 9-axis mode the seed and the mag reference are captured
  // together, after calibration, while the board is still.
  Quaternion cq;
  float mag_body[3] = {0.0f, 0.0f, 0.0f};

  if (kFuseMagnetometer) {
    if (!RunCalibration()) {
      UART_Print("WARNING: continuing UNCALIBRATED -- expect yaw errors.\r\n");
    }

    if (!CaptureStillReference(mag_body, &cq)) {
      UART_Print("=== IMU never held still for the heading reference; halting ===\r\n");
      while (1) {
        delay(1000);
      }
    }
  } else {
    cq = ChipQuaternion();
  }

  float cq_norm = sqrtf(cq.w * cq.w + cq.x * cq.x + cq.y * cq.y + cq.z * cq.z);
  if (cq_norm < 0.5f) {
    UART_Print("=== chip fusion output never arrived; cannot seed EKF ===\r\n");
    while (1) {
      delay(1000);
    }
  }

  quat_t q0 = {cq.w, cq.x, cq.y, cq.z};
  ekf_ahrs_set_attitude(&ekf, q0);

  if (kFuseMagnetometer) {
    // Now that the attitude is known, rotate the body-frame mag sample into
    // the nav frame to get a valid heading reference. Doing this the other
    // way round (or skipping the rotation entirely) offsets heading by
    // exactly the startup attitude, which is what made yaw disagree by
    // ~165 deg.
    ekf_ahrs_set_mag_ref_from_body(&ekf, mag_body);

    // Sanity check for the user: dip is fixed by location (NOAA's field
    // calculator gives the local value; ~70 deg in southern Ontario) and
    // should come out the same on every run once calibration is good.
    float dip_deg = asinf(-ekf.mag_ref[2]) * 180.0f / 3.14159265f;

    char msg[200];
    snprintf(msg, sizeof(msg),
             "mag ref: |B|=%.1f uT  nav dir=(%.3f, %.3f, %.3f)  dip=%.1f deg\r\n",
             ekf.mag_ref_norm, ekf.mag_ref[0], ekf.mag_ref[1], ekf.mag_ref[2],
             dip_deg);
    UART_Print(msg);
  }

  // Flush the backlog that built up while we were collecting samples, so the
  // filter doesn't start by replaying a burst of stale reports.
  for (int i = 0; i < 20; i++) {
    imu.update();
  }

  last_gyro_ts = 0;
  ekf_initialized = true;
  stream_start_ms = HAL_GetTick();
  imu.setReportHandler(OnImuReport, nullptr);

  UART_Print("EKF initialized. Starting comparison stream...\r\n");
}

void loop() {
  // All filtering happens inside OnImuReport() as reports arrive. This loop
  // only has to keep the hub drained and print occasionally, so it is free to
  // run as fast as the I2C transport allows.
  imu.update();

  if (!ekf_initialized) return;

  static uint32_t last_print_ms = 0;

  uint32_t now = HAL_GetTick();
  if ((now - last_print_ms) < 500) return;

  uint32_t elapsed_ms = now - last_print_ms;
  last_print_ms = now;
  heartbeat_toggle();

  float our_roll, our_pitch, our_yaw;
  ekf_ahrs_get_euler(&ekf, &our_roll, &our_pitch, &our_yaw);

  // Chip's own built-in fusion, for comparison only. Our EKF's quaternion is
  // NOT reset to this -- it runs freely on its own after the startup seed.
  Quaternion cq = ChipQuaternion();
  quat_t chip_q;
  chip_q.w = cq.w; chip_q.x = cq.x; chip_q.y = cq.y; chip_q.z = cq.z;
  float chip_roll, chip_pitch, chip_yaw;
  quat_to_euler(chip_q, &chip_roll, &chip_pitch, &chip_yaw);

  const float kRad2Deg = 180.0f / 3.14159265f;

  // Euler angles wrap, so a raw difference reports nonsense like -352 deg for
  // what is really a -8 deg disagreement. Fold into (-180, 180].
  float roll_err = wrap_deg((our_roll - chip_roll) * kRad2Deg);
  float pitch_err = wrap_deg((our_pitch - chip_pitch) * kRad2Deg);
  float yaw_err = wrap_deg((our_yaw - chip_yaw) * kRad2Deg);

  char errMsg[128];
  snprintf(errMsg, sizeof(errMsg),
           "ERR  t=%5.1fs  roll=%+.2f  pitch=%+.2f  yaw=%+.2f\r\n",
           (now - stream_start_ms) / 1000.0f,
           roll_err, pitch_err, yaw_err);
  UART_Print(errMsg);

  char msg[192];
  snprintf(msg, sizeof(msg),
           "OUR EKF  roll=%.1f pitch=%.1f yaw=%.1f  |  %s  roll=%.1f pitch=%.1f yaw=%.1f\r\n",
           our_roll * kRad2Deg, our_pitch * kRad2Deg, our_yaw * kRad2Deg,
           kFuseMagnetometer ? "CHIP" : "CHIP(game)",
           chip_roll * kRad2Deg, chip_pitch * kRad2Deg, chip_yaw * kRad2Deg);
  UART_Print(msg);

  // Transport health, printed only when something is wrong: the filter is
  // being starved of gyro samples, or the I2C link / hub had to be recovered.
  // Either one makes attitude drift under motion however the filter is tuned.
  static unsigned long last_recov = 0, last_bus = 0;
  static bool first_window = true; // spans all of setup(); rate meaningless
  unsigned long predict_rate = predict_count * 1000UL / elapsed_ms;
  unsigned long recov = imu.recoveries();
  unsigned long bus = SH2_HAL_BusRecoveries();
  predict_count = 0;

  if (!first_window &&
      (predict_rate < 80 || recov != last_recov || bus != last_bus)) {
    char warnMsg[128];
    snprintf(warnMsg, sizeof(warnMsg),
             "WARN  predict=%lu/s (expect ~100)  hub_resets=%lu  i2c_recoveries=%lu\r\n",
             predict_rate, recov, bus);
    UART_Print(warnMsg);
  }
  last_recov = recov;
  last_bus = bus;
  first_window = false;

  // Magnetometer cross-check. hdg_err is the correction the MAGNETOMETER wants
  // applied to our yaw, independent of the chip's fusion. ERR yaw is (ours -
  // chip), so compare hdg_err against -ERR yaw:
  //   - hdg_err ~ -ERR yaw -> the mag agrees with the chip; our filter is slow
  //                      or mis-weighted in following it.
  //   - mag ~0 but ERR yaw large -> the mag agrees with US, so the field
  //                      itself shifted relative to mag_ref (the chip's
  //                      dynamic mag calibration updating, or local iron).
  // acc is the chip's own 0..3 accuracy estimate for the calibrated field.
  if (kFuseMagnetometer) {
    sh2_SensorValue_t ms = imu.getMagnetometer();
    float mag[3] = {ms.un.magneticField.x, ms.un.magneticField.y, ms.un.magneticField.z};
    float mag_norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    float mag_err = 0.0f;
    int have = ekf_ahrs_mag_heading_error(&ekf, mag, &mag_err);

    char magMsg[160];
    snprintf(magMsg, sizeof(magMsg),
             "MAG  hdg_err=%+.2f%s  |B|=%.1fuT (ref %.1f)  acc=%u  rejected=%lu\r\n",
             have ? mag_err * kRad2Deg : 0.0f, have ? "" : "(n/a)",
             mag_norm, ekf.mag_ref_norm, (unsigned)Accuracy(ms),
             ekf.mag_rejects);
    UART_Print(magMsg);
  }
}
