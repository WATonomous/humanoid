#ifndef BNO085_H
#define BNO085_H

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

class BNO085 {
public:
  BNO085();

  bool begin();

  void update();

  bool enableRotationVector(uint32_t interval_us = 10000);
  bool enableAccelerometer(uint32_t interval_us = 10000);
  bool enableGyroscope(uint32_t interval_us = 10000);
  bool enableMagnetometer(uint32_t interval_us = 20000);

  Quaternion getQuaternion() const;
  sh2_SensorValue_t getAccelerometer() const;
  sh2_SensorValue_t getGyroscope() const;
  sh2_SensorValue_t getMagnetometer() const;

  // Diagnostics - not part of normal operation, just for bring-up debugging
  bool rotationVectorEnableOk() const { return rotationVectorEnableOk_; }
  uint32_t totalEventsReceived() const { return totalEvents_; }
  uint32_t decodeFailures() const { return decodeFailures_; }
  uint32_t rotationVectorEventsReceived() const { return rotationVectorEvents_; }

private:
  static void SensorCallback(void* cookie, sh2_SensorEvent_t* event);

  static void EventCallback(void* cookie, sh2_AsyncEvent_t* event);

  void handleSensorEvent(sh2_SensorEvent_t* event);

  sh2_SensorValue_t rotationVector;
  sh2_SensorValue_t accelerometer;
  sh2_SensorValue_t gyroscope;
  sh2_SensorValue_t magnetometer;

  bool rotationVectorEnableOk_ = false;
  uint32_t totalEvents_ = 0;
  uint32_t decodeFailures_ = 0;
  uint32_t rotationVectorEvents_ = 0;
};

#endif