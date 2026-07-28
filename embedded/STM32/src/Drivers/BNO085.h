#ifndef BNO085_H
#define BNO085_H

#include "sh2.h"
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

  Quaternion getQuaternion() const;
  sh2_SensorValue_t getAccelerometer() const;
  sh2_SensorValue_t getGyroscope() const;

private:
  static void SensorCallback(void* cookie, sh2_SensorEvent_t* event);

  static void EventCallback(void* cookie, sh2_AsyncEvent_t* event);

  void handleSensorEvent(sh2_SensorEvent_t* event);

  sh2_SensorValue_t rotationVector;
  sh2_SensorValue_t accelerometer;
  sh2_SensorValue_t gyroscope;
};

#endif