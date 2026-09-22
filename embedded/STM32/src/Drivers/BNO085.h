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
  // Invoked once for every decoded report, in arrival order, from inside
  // update(). Lets a consumer step a filter at each sensor's own rate and
  // timestamp rather than at control-loop rate.
  typedef void (*ReportHandler)(const sh2_SensorValue_t& value, void* ctx);

  BNO085();

  bool begin();

  // Drains every report the hub has queued. See the definition for why one
  // sh2_service() call per control loop is not enough.
  void update();

  void setReportHandler(ReportHandler handler, void* ctx);

  bool enableRotationVector(uint32_t interval_us = 10000);
  bool disableRotationVector();
  // Hub's own gyro + accel fusion, no magnetometer. Yaw is relative to
  // wherever it started and drifts slowly, but is immune to magnetic
  // distortion -- the like-for-like reference for a mag-less filter.
  bool enableGameRotationVector(uint32_t interval_us = 10000);
  bool enableAccelerometer(uint32_t interval_us = 10000);
  bool enableGyroscope(uint32_t interval_us = 10000);
  bool enableMagnetometer(uint32_t interval_us = 20000);

  Quaternion getQuaternion() const;
  Quaternion getGameQuaternion() const;

  // Hub's own estimate of the rotation vector's heading accuracy, radians.
  float headingAccuracy() const;

  // --- Dynamic calibration (the hub's "DCD") ---
  // The hub continuously estimates accel/gyro/mag calibration while it runs
  // and applies it to the *_CALIBRATED reports. These choose which sensors it
  // calibrates and control when the result is written to its flash, so a
  // good calibration survives a power cycle. Both settings are re-applied by
  // recover() after a hub reset.
  bool setCalibrationConfig(uint8_t sensors); // mask of SH2_CAL_* flags
  bool setCalibrationAutoSave(bool enabled);
  bool saveCalibration();
  sh2_SensorValue_t getAccelerometer() const;
  sh2_SensorValue_t getGyroscope() const;
  sh2_SensorValue_t getMagnetometer() const;

  // Force a full transport + hub reset and re-apply the enabled reports.
  // update() calls this on its own when the stream stalls.
  bool recover();

  // Diagnostics - not part of normal operation, just for bring-up debugging
  uint32_t recoveries() const { return recoveries_; }
  bool rotationVectorEnableOk() const { return rotationVectorEnableOk_; }
  uint32_t totalEventsReceived() const { return totalEvents_; }
  uint32_t decodeFailures() const { return decodeFailures_; }
  uint32_t rotationVectorEventsReceived() const { return rotationVectorEvents_; }

private:
  static void SensorCallback(void* cookie, sh2_SensorEvent_t* event);

  static void EventCallback(void* cookie, sh2_AsyncEvent_t* event);

  void handleSensorEvent(sh2_SensorEvent_t* event);

  sh2_SensorValue_t rotationVector;
  sh2_SensorValue_t gameRotationVector;
  sh2_SensorValue_t accelerometer;
  sh2_SensorValue_t gyroscope;
  sh2_SensorValue_t magnetometer;

  // Applies a report config AND records it, so recover() can re-apply
  // everything that was enabled before the hub was reset.
  bool enableSensor(uint8_t sensorId, uint32_t interval_us);

  static const int kMaxEnabledSensors = 6;

  struct EnabledSensor {
    uint8_t id;
    uint32_t interval_us;
  };

  EnabledSensor enabled_[kMaxEnabledSensors] = {};
  int enabledCount_ = 0;

  // Calibration settings to restore after a hub reset (0 / false = never set).
  uint8_t calConfig_ = 0;
  bool calAutoSaveSet_ = false;
  bool calAutoSave_ = true;

  // Wall-clock of the last successfully decoded report. Zero disarms the
  // stall watchdog (before begin() has succeeded).
  uint32_t lastReportMs_ = 0;
  uint32_t recoveries_ = 0;

  // How long the report stream may be silent before the hub is reset. The
  // slowest enabled report is 20 Hz, so 500 ms is ~10 missed reports.
  static const uint32_t kStallTimeoutMs = 500;

  ReportHandler reportHandler_ = nullptr;
  void* reportCtx_ = nullptr;

  bool rotationVectorEnableOk_ = false;
  uint32_t totalEvents_ = 0;
  uint32_t decodeFailures_ = 0;
  uint32_t rotationVectorEvents_ = 0;
};

#endif