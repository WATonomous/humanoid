#include "BNO085.h"

#include "sh2_hal_stm32.h"
#include "stm32g4xx_hal.h"
#include <cstring>

BNO085::BNO085() {
  memset(&rotationVector, 0, sizeof(rotationVector));
  memset(&gameRotationVector, 0, sizeof(gameRotationVector));
  memset(&accelerometer, 0, sizeof(accelerometer));
  memset(&gyroscope, 0, sizeof(gyroscope));
  memset(&magnetometer, 0, sizeof(magnetometer));
}

bool BNO085::begin() {
  int status = sh2_open(SH2_HAL_GetInstance(), EventCallback, this);

  if (status != SH2_OK)
    return false;

  sh2_setSensorCallback(SensorCallback, this);

  // Give the sensor hub time to finish its initial power-on handshake
  // (product ID / advertisement exchange) before asking it to enable
  // a sensor report. Without this, enableRotationVector() below can
  // fail even though the transport itself is working fine, because
  // the SH2 library hasn't finished learning the hub's channel/app
  // mapping yet.
  for (int i = 0; i < 20; i++) {
    sh2_service();
    HAL_Delay(20);
  }

  rotationVectorEnableOk_ = enableRotationVector();

  // Arm the stall watchdog only now that the transport is up.
  lastReportMs_ = HAL_GetTick();

  return true;
}

bool BNO085::recover() {
  recoveries_++;

  // Set this first: if recovery itself fails, the watchdog waits a full
  // timeout before trying again rather than spinning on resets.
  lastReportMs_ = HAL_GetTick();

  sh2_close();

  // sh2_open() -> SH2_Open() re-inits the I2C peripheral and pulses the
  // BNO085's RST line. The reset is what clears a hub that stopped asserting
  // INT, and it also releases SDA if the hub was left holding the bus.
  if (sh2_open(SH2_HAL_GetInstance(), EventCallback, this) != SH2_OK) {
    return false;
  }

  sh2_setSensorCallback(SensorCallback, this);

  // Same settle as begin(): the hub must finish its advertisement exchange
  // before it will accept a sensor config.
  for (int i = 0; i < 20; i++) {
    sh2_service();
    HAL_Delay(20);
  }

  // Re-apply whatever was enabled before the stall. enableSensor() updates
  // entries in place, so re-running it over this list doesn't grow it.
  bool ok = true;

  for (int i = 0; i < enabledCount_; i++) {
    if (!enableSensor(enabled_[i].id, enabled_[i].interval_us)) {
      ok = false;
    }
  }

  // Calibration config is runtime state and reverts to the hub's defaults on
  // reset. (The calibration DATA itself is reloaded from the hub's flash.)
  if (calConfig_ != 0 && sh2_setCalConfig(calConfig_) != SH2_OK) {
    ok = false;
  }
  if (calAutoSaveSet_ && sh2_setDcdAutoSave(calAutoSave_) != SH2_OK) {
    ok = false;
  }

  lastReportMs_ = HAL_GetTick();

  return ok;
}

void BNO085::update() {
  // shtp_service() performs exactly one hal->read(), i.e. one sh2_service()
  // call consumes at most ONE SHTP packet. With accel + gyro + mag enabled the
  // hub emits a few hundred reports/s, so servicing once per control loop left
  // its FIFO permanently backed up and delivered samples tens of ms stale.
  //
  // Drain until the hub deasserts INT. The cap bounds worst-case time spent
  // here if INT ever sticks low -- e.g. a packet too large for the SHTP input
  // buffer, which SH2_Read() has no choice but to drop.
  const int kMaxPacketsPerUpdate = 32;

  // Service once unconditionally before testing INT: shtp_service() also
  // drives the SHTP advertisement handshake, which must still run when the
  // hub has no sensor data pending. With no data, SH2_Read() sees INT high
  // and returns immediately without touching the bus.
  for (int i = 0; i < kMaxPacketsPerUpdate; i++) {
    sh2_service();

    // A failed transfer leaves INT asserted, so without this the drain would
    // retry the same broken read up to kMaxPacketsPerUpdate times, each
    // paying its own I2C timeout. Back off and let the watchdog decide.
    if (SH2_HAL_TakeTransferError())
      break;

    if (!SH2_HAL_INT_IsAsserted())
      break;
  }

  // Stall watchdog.
  //
  // The hub can stop reporting outright -- an aborted transfer (a connector
  // glitch while the board is being handled is enough) can leave its SHTP
  // state machine waiting to finish a transaction that never completes, and
  // it then stops asserting INT. Since SH2_Read() returns immediately while
  // INT is high, nothing in this driver would ever touch the bus again and
  // the stream would stay dead until the MCU was reset.
  if (lastReportMs_ != 0 &&
      (HAL_GetTick() - lastReportMs_) > kStallTimeoutMs) {
    recover();
  }
}

void BNO085::setReportHandler(ReportHandler handler, void* ctx) {
  reportHandler_ = handler;
  reportCtx_ = ctx;
}

bool BNO085::enableSensor(uint8_t sensorId, uint32_t interval_us) {
  sh2_SensorConfig_t config;

  memset(&config, 0, sizeof(config));

  config.reportInterval_us = interval_us;

  if (sh2_setSensorConfig(sensorId, &config) != SH2_OK)
    return false;

  // Remember it so recover() can re-apply it after a hub reset. Entries are
  // updated in place so re-enabling never grows the list.
  for (int i = 0; i < enabledCount_; i++) {
    if (enabled_[i].id == sensorId) {
      enabled_[i].interval_us = interval_us;
      return true;
    }
  }

  if (enabledCount_ < kMaxEnabledSensors) {
    enabled_[enabledCount_].id = sensorId;
    enabled_[enabledCount_].interval_us = interval_us;
    enabledCount_++;
  }

  return true;
}

bool BNO085::enableRotationVector(uint32_t interval_us) {
  return enableSensor(SH2_ROTATION_VECTOR, interval_us);
}

// A report interval of 0 tells the hub to stop producing the report. It is
// recorded like any other config, so recover() keeps it disabled.
bool BNO085::disableRotationVector() {
  return enableSensor(SH2_ROTATION_VECTOR, 0);
}

bool BNO085::enableGameRotationVector(uint32_t interval_us) {
  return enableSensor(SH2_GAME_ROTATION_VECTOR, interval_us);
}

bool BNO085::enableAccelerometer(uint32_t interval_us) {
  return enableSensor(SH2_ACCELEROMETER, interval_us);
}

bool BNO085::enableGyroscope(uint32_t interval_us) {
  return enableSensor(SH2_GYROSCOPE_CALIBRATED, interval_us);
}

bool BNO085::enableMagnetometer(uint32_t interval_us) {
  return enableSensor(SH2_MAGNETIC_FIELD_CALIBRATED, interval_us);
}


Quaternion BNO085::getQuaternion() const {
  Quaternion q;

  q.w = rotationVector.un.rotationVector.real;
  q.x = rotationVector.un.rotationVector.i;
  q.y = rotationVector.un.rotationVector.j;
  q.z = rotationVector.un.rotationVector.k;

  return q;
}

Quaternion BNO085::getGameQuaternion() const {
  Quaternion q;

  q.w = gameRotationVector.un.gameRotationVector.real;
  q.x = gameRotationVector.un.gameRotationVector.i;
  q.y = gameRotationVector.un.gameRotationVector.j;
  q.z = gameRotationVector.un.gameRotationVector.k;

  return q;
}

float BNO085::headingAccuracy() const {
  return rotationVector.un.rotationVector.accuracy;
}

bool BNO085::setCalibrationConfig(uint8_t sensors) {
  if (sh2_setCalConfig(sensors) != SH2_OK)
    return false;

  calConfig_ = sensors;
  return true;
}

bool BNO085::setCalibrationAutoSave(bool enabled) {
  if (sh2_setDcdAutoSave(enabled) != SH2_OK)
    return false;

  calAutoSaveSet_ = true;
  calAutoSave_ = enabled;
  return true;
}

bool BNO085::saveCalibration() {
  return sh2_saveDcdNow() == SH2_OK;
}

sh2_SensorValue_t BNO085::getAccelerometer() const {
  return accelerometer;
}

sh2_SensorValue_t BNO085::getGyroscope() const {
  return gyroscope;
}

sh2_SensorValue_t BNO085::getMagnetometer() const {
  return magnetometer;
}

void BNO085::SensorCallback(void* cookie, sh2_SensorEvent_t* event) {
  BNO085* imu = static_cast<BNO085*>(cookie);

  imu->handleSensorEvent(event);
}

void BNO085::EventCallback(void* cookie, sh2_AsyncEvent_t* event) {
  (void)cookie;
  (void)event;
}

void BNO085::handleSensorEvent(sh2_SensorEvent_t* event) {
  sh2_SensorValue_t value;

  totalEvents_++;
  lastReportMs_ = HAL_GetTick();

  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
    decodeFailures_++;
    return;
  }

  switch (value.sensorId) {
  case SH2_ROTATION_VECTOR:
    rotationVector = value;
    rotationVectorEvents_++;
    break;

  case SH2_GAME_ROTATION_VECTOR:
    gameRotationVector = value;
    break;

  case SH2_ACCELEROMETER:
    accelerometer = value;
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    gyroscope = value;
    break;

  case SH2_MAGNETIC_FIELD_CALIBRATED:
    magnetometer = value;
    break;

  default:
    break;
  }

  // Caches are refreshed first, so a handler can also read the other sensors'
  // most recent values while servicing this report.
  if (reportHandler_ != nullptr) {
    reportHandler_(value, reportCtx_);
  }
}