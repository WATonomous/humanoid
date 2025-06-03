#include <Arduino.h>
#include <SPI.h>
#include "MT6835.h"
#include "foc_utils.h"  // you already had this

// XIAO ESP32-S3 SPI pins (example if you wired CS to D7/GPIO6):
constexpr uint8_t PIN_MOSI = 9;  // D10 = GPIO9
constexpr uint8_t PIN_MISO = 8;  // D9  = GPIO8
constexpr uint8_t PIN_SCK  = 7;  // D8  = GPIO7
constexpr uint8_t PIN_CS   = 6;  // D7  = GPIO6

SPISettings encSPI(1000000, MSBFIRST, SPI_MODE3);
MT6835 encoder(encSPI, PIN_CS);

void setup() {
  Serial.begin(115200);

  // Wait a brief moment for the USB-Serial to enumerate:
  delay(100);

  Serial.println(F(">> setup start"));

  Serial.println(F("   Calling SPI.begin"));
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  Serial.println(F("   SPI.begin OK"));

  Serial.println(F("   Calling encoder.init"));
  encoder.init(&SPI);
  Serial.println(F("   encoder.init returned"));

  // Turn on CRC checking if you like:
  encoder.checkcrc = true;

  // Print calibration / config registers:
  uint8_t cal = encoder.getCalibrationStatus();
  Serial.print(F("Calibration Status: "));
  switch (cal) {
    case 0: Serial.println(F("Not Calibrated"));    break;
    case 1: Serial.println(F("Calibrating"));        break;
    case 2: Serial.println(F("Calibrated"));         break;
    case 3: Serial.println(F("Calibration Error"));  break;
    default: Serial.println(F("Unknown"));           break;
  }

  Serial.print(F("Default Bandwidth: "));
  Serial.println(encoder.getBandwidth());
  Serial.print(F("Default Hysteresis: "));
  Serial.println(encoder.getHysteresis());
  Serial.print(F("Default Rotation Dir: "));
  Serial.println(encoder.getRotationDirection());
  Serial.print(F("ABZ Enabled? "));
  Serial.println(encoder.isABZEnabled() ? F("Yes") : F("No"));
  Serial.print(F("ABZ Resolution: "));
  Serial.println(encoder.getABZResolution());
  Serial.print(F("AB Swapped? "));
  Serial.println(encoder.isABSwapped() ? F("Yes") : F("No"));

  Serial.println(F(">> setup complete\n"));
}

void loop() {
  Serial.println(F(">> loop start"));

  // Read 21-bit raw angle:
  uint32_t raw = encoder.readRawAngle21();
  Serial.print(F("   raw = 0x"));
  Serial.println(raw, HEX);

  // Read status bits and check for CRC error:
  uint8_t status = encoder.getStatus();
  Serial.print(F("   status bits = 0b"));
  for (int i = 2; i >= 0; --i) {
    Serial.print((status >> i) & 1);
  }
  Serial.println();
  bool crcErr = (status & MT6835_CRC_ERROR);
  if (crcErr) {
    Serial.println(F("   CRC ERROR DETECTED"));
  }

  // Convert to radians (returns –1.0 on CRC error):
  float angleRad = encoder.getCurrentAngle();
  Serial.print(F("   angleRad = "));
  Serial.println(angleRad, 6);

  Serial.println(F(">> loop end\n"));
  delay(1000);
}
