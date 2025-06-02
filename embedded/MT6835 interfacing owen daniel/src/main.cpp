#include <Arduino.h>
#include <SPI.h>
#include "MT6835.h"
#include "foc_utils.h"

// XIAO ESP32‑S3 pin mapping for SPI:
constexpr uint8_t PIN_MOSI = 9;   // D10 = GPIO9
constexpr uint8_t PIN_MISO = 8;   // D9  = GPIO8
constexpr uint8_t PIN_SCK  = 7;   // D8  = GPIO7
constexpr uint8_t PIN_CS   = 6;   // D7  = GPIO6  (example)

SPISettings encSPI(1000000, MSBFIRST, SPI_MODE3);
MT6835 encoder(encSPI, PIN_CS);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Tell the SPI bus exactly which pins to use:
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Initialize the encoder on that SPI bus:
  encoder.init(&SPI);

  // (Remaining setup as before…)
  encoder.checkcrc = true;
  uint8_t cal = encoder.getCalibrationStatus();
  Serial.print("Calibration Status: ");
  switch (cal) {
    case 0: Serial.println("Not Calibrated"); break;
    case 1: Serial.println("Calibrating");    break;
    case 2: Serial.println("Calibrated");     break;
    case 3: Serial.println("Calibration Error"); break;
    default: Serial.println("Unknown");        break;
  }

  Serial.print("Default Bandwidth: ");
  Serial.println(encoder.getBandwidth());
  Serial.print("Default Hysteresis: ");
  Serial.println(encoder.getHysteresis());
  Serial.print("Default Rotation Dir: ");
  Serial.println(encoder.getRotationDirection());

  Serial.print("ABZ Enabled? ");
  Serial.println(encoder.isABZEnabled() ? "Yes" : "No");
  Serial.print("ABZ Resolution: ");
  Serial.println(encoder.getABZResolution());
  Serial.print("AB Swapped? ");
  Serial.println(encoder.isABSwapped() ? "Yes" : "No");

  Serial.println("\nStarting main loop...\n");
}

void loop() {
  uint32_t raw = encoder.readRawAngle21();
  uint8_t status = encoder.getStatus();
  bool crcErr = (status & MT6835_CRC_ERROR);
  float angleRad = encoder.getCurrentAngle();

  Serial.print("Raw=0x");
  Serial.print(raw, HEX);
  Serial.print("   Status=0b");
  for (int i = 2; i >= 0; --i) {
    Serial.print((status >> i) & 0x01);
  }
  if (crcErr) {
    Serial.print(" (CRC ERR)");
  }
  Serial.print("   Angle=");
  if (angleRad < 0) {
    Serial.print("ERR");
  } else {
    Serial.print(angleRad, 6);
    Serial.print(" rad  (");
    Serial.print(angleRad * 180.0 / 3.14159265, 2);
    Serial.print("°)");
  }
  Serial.println();

  static uint32_t lastToggle = 0;
  if (millis() - lastToggle > 5000) {
    lastToggle = millis();
    bool currentlyOn = encoder.isABZEnabled();
    encoder.setABZEnabled(!currentlyOn);
    Serial.print(">>> ABZ now ");
    Serial.println(currentlyOn ? "DISABLED" : "ENABLED");
  }

  delay(100);
}
