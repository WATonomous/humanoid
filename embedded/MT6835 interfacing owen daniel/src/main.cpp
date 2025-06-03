#include <Arduino.h>
#include <SPI.h>
#include "MT6835.h"
#include "foc_utils.h"

// Use the same pin definitions as the manual test:
constexpr uint8_t PIN_MOSI = 9;
constexpr uint8_t PIN_MISO = 8;
constexpr uint8_t PIN_SCK  = 7;
constexpr uint8_t PIN_CS   = 5;

SPISettings encSPI(1000000, MSBFIRST, SPI_MODE3);
MT6835 encoder(encSPI, PIN_CS);

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  encoder.init(&SPI);
  encoder.checkcrc = true;
  Serial.println("MT6835 initialized.");
}

void loop() {
  uint32_t raw    = encoder.readRawAngle21();
  uint8_t  status = encoder.getStatus();

  bool crcError   = (status & MT6835_CRC_ERROR) != 0;
  bool magWarn    = (status & 0b010) != 0;  // bit 1
  bool notReady   = (status & 0b001) != 0;  // bit 0

  Serial.print("Raw = 0x");
  Serial.print(raw, HEX);
  Serial.print("   ");

  if (status == 0) {
    Serial.print("Status = OK");
  } else {
    Serial.print("Status =");
    if (crcError) Serial.print(" [CRC Error]");
    if (magWarn)  Serial.print(" [Magnet Warning]");
    if (notReady) Serial.print(" [Not Ready]");
  }

  float angleRad = encoder.getCurrentAngle();
  Serial.print("   Angle = ");
  if (angleRad < 0) {
    Serial.print("ERR");
  } else {
    Serial.print(angleRad, 6);
    Serial.print(" rad  (");
    Serial.print(angleRad * 180.0 / 3.14159265, 2);
    Serial.print("°)");
  }
  Serial.println();
  delay(200);
}
