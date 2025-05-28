#include <Arduino.h>
#include "MT6835.h"          // your low-level driver

constexpr uint8_t CS_PIN = 10;
SPISettings encSPI(1000000, MSBFIRST, SPI_MODE3);
MT6835 encoder(encSPI, CS_PIN);

void setup() {
  Serial.begin(115200);
  encoder.init(&SPI);
}

void loop() {
  float angle = encoder.getCurrentAngle();   // radians
  Serial.println(angle, 6);
  delay(10);
}
