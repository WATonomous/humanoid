#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

#include "encoders/mt6835/MagneticSensorMT6835.h"

#define SENSOR_nCS PB6

SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(SENSOR_nCS, myMT6835SPISettings);

long ts;

void setup() {
    sensor.init();
    ts = millis();
}

void loop() {
    sensor.update();
    long now = millis();
    if (now - ts > 1000) {
        ts = now;
        SimpleFOCDebug::print("A: ");
        SimpleFOCDebug::print(sensor.getAngle());
        SimpleFOCDebug::print(" V: ");
        SimpleFOCDebug::println(sensor.getVelocity());
    }
    delay(10);
}
