#include "BNO085.h"

#include "sh2_hal_stm32.h"

#include <cstring>

BNO085::BNO085() {
  memset(&rotationVector, 0, sizeof(rotationVector));
  memset(&accelerometer, 0, sizeof(accelerometer));
  memset(&gyroscope, 0, sizeof(gyroscope));
}

bool BNO085::begin() {
  int status = sh2_open(SH2_HAL_GetInstance(), EventCallback, this);

  if (status != SH2_OK)
    return false;

  sh2_setSensorCallback(SensorCallback, this);

  enableRotationVector();

  return true;
}

void BNO085::update() {
  sh2_service();
}

bool BNO085::enableRotationVector(uint32_t interval_us) {
  sh2_SensorConfig_t config;

  memset(&config, 0, sizeof(config));

  config.reportInterval_us = interval_us;

  return sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config) == SH2_OK;
}

bool BNO085::enableAccelerometer(uint32_t interval_us) {
  sh2_SensorConfig_t config;

  memset(&config, 0, sizeof(config));

  config.reportInterval_us = interval_us;

  return sh2_setSensorConfig(SH2_ACCELEROMETER, &config) == SH2_OK;
}

bool BNO085::enableGyroscope(uint32_t interval_us) {
  sh2_SensorConfig_t config;

  memset(&config, 0, sizeof(config));

  config.reportInterval_us = interval_us;

  return sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config) == SH2_OK;
}

Quaternion BNO085::getQuaternion() const {
  return rotationVector;
}

sh2_SensorValue_t BNO085::getAccelerometer() const {
  return accelerometer;
}

sh2_SensorValue_t BNO085::getGyroscope() const {
  return gyroscope;
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

  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
    return;
  }

  switch (value.sensorId) {
  case SH2_ROTATION_VECTOR:
    rotationVector = value;
    break;

  case SH2_ACCELEROMETER:
    accelerometer = value;
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    gyroscope = value;
    break;

  default:
    break;
  }
}