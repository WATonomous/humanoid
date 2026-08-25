#include "BNO085.h"

#include "sh2_hal_stm32.h"
#include "stm32g4xx_hal.h"
#include <cstring>

BNO085::BNO085() {
  memset(&rotationVector, 0, sizeof(rotationVector));
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

bool BNO085::enableMagnetometer(uint32_t interval_us) {
  sh2_SensorConfig_t config;

  memset(&config, 0, sizeof(config));

  config.reportInterval_us = interval_us;

  return sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &config) == SH2_OK;
}

Quaternion BNO085::getQuaternion() const {
  Quaternion q;

  q.w = rotationVector.un.rotationVector.real;
  q.x = rotationVector.un.rotationVector.i;
  q.y = rotationVector.un.rotationVector.j;
  q.z = rotationVector.un.rotationVector.k;

  return q;
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

  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
    decodeFailures_++;
    return;
  }

  switch (value.sensorId) {
  case SH2_ROTATION_VECTOR:
    rotationVector = value;
    rotationVectorEvents_++;
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
}