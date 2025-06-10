#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <MagneticSensorMT6835.h>

constexpr uint8_t PIN_CS = PB0;     // choose any free output-capable pin

SPISettings mt6835SPI(500000, MSBFIRST, SPI_MODE3);   // start at 500 kHz
MagneticSensorMT6835 sensor(PIN_CS, mt6835SPI);       // **only 2 args**

void setup() {
  Serial.begin(115200);

  // Point the *global* SPI object at the real hardware pins for SPI-1
  // (works with the official Arduino_Core_STM32)
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  sensor.init();
}

void loop() {
  sensor.update();
  Serial.print(F("Angle [rad]: "));
    Serial.print(sensor.getAngle(), 6);
    Serial.print(F("   Velocity [rad/s]: "));
    Serial.println(sensor.getVelocity(), 4);
  delay(10);
}