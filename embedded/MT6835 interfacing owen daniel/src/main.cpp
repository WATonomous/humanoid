/***************************************************************************
 *  SimpleFOC – MT6835 SPI encoder demo (ESP32-S3)                         *
 *  Uses ONLY the core "Simple FOC" library                                *
 ***************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>                       // core library – nothing else
#include <MT6835.h>
#include <MagneticSensorMT6835.h>
// #include "encoders/mt6835/MagneticSensorMT6835.h"   // this header lives in
//                                                     // SimpleFOC itself

// ---------- board-specific pin map ----------------------------------------
constexpr uint8_t PIN_MOSI = 9;   // XIAO D10 (GPIO  9)
constexpr uint8_t PIN_MISO = 8;   // XIAO D9  (GPIO  8)
constexpr uint8_t PIN_SCK  = 7;   // XIAO D8  (GPIO  7)
constexpr uint8_t PIN_CS   = 5;   // XIAO D7  (GPIO  6)  – use your CS pin

// ---------- SimpleFOC sensor object ---------------------------------------
SPISettings mt6835SPI(1000000, MSBFIRST, SPI_MODE3);
MagneticSensorMT6835 sensor(PIN_CS, mt6835SPI);

void setup() {
  Serial.begin(115200);
  delay(200);                                      // let USB enumerate

  // ESP32-S3 hardware-SPI bus on explicit pins
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  sensor.init();                                   // initialise the encoder

  Serial.println(F("MT6835 + SimpleFOC ready"));
}

void loop() {
  sensor.update();                                 // refresh angle + velocity

  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (now - t0 >= 1000) {                          // print once per second
    t0 = now;
    Serial.print(F("Angle [rad]: "));
    Serial.print(sensor.getAngle(), 6);
    Serial.print(F("   Velocity [rad/s]: "));
    Serial.println(sensor.getVelocity(), 4);
  }

  delay(10);                                       // 100 Hz update rate
}
