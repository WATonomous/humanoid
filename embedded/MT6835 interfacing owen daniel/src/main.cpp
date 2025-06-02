#include <Arduino.h>
#include <SPI.h>
#include "MT6835.h"            
#include "foc_utils.h"         


// // MT6835       → XIAO ESP32‑S3
// ─────────────┬─────────────────────
// MOSI         │ MOSI / GPIO9
// MISO         │ MISO / GPIO8
// SCK          │ SCK / GPIO7
// CS           │ D10 / GPIO10
// VDD          │ 3V3
// GND          │ GND

constexpr uint8_t CS_PIN = 10;  
SPISettings encSPI(1000000, MSBFIRST, SPI_MODE3);
MT6835 encoder(encSPI, CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  encoder.init(&SPI);

  // Optional: enable CRC checking on angle reads
  encoder.checkcrc = true;

  // Print initial calibration status
  uint8_t cal = encoder.getCalibrationStatus();
  Serial.print("Calibration Status: ");
  switch (cal) {
    case 0: Serial.println("Not Calibrated"); break;
    case 1: Serial.println("Calibrating"); break;
    case 2: Serial.println("Calibrated"); break;
    case 3: Serial.println("Calibration Error"); break;
    default: Serial.println("Unknown"); break;
  }

  Serial.print("Default Bandwidth: ");
  Serial.println(encoder.getBandwidth());
  Serial.print("Default Hysteresis: ");
  Serial.println(encoder.getHysteresis());
  Serial.print("Default Rotation Dir: ");
  Serial.println(encoder.getRotationDirection());

  // Optional: print ABZ settings
  Serial.print("ABZ Enabled? ");
  Serial.println(encoder.isABZEnabled() ? "Yes" : "No");
  Serial.print("ABZ Resolution: ");
  Serial.println(encoder.getABZResolution());
  Serial.print("AB Swapped? ");
  Serial.println(encoder.isABSwapped() ? "Yes" : "No");

  Serial.println("\nStarting main loop...\n");
}

void loop() {
  // 1) Read raw 21-bit angle
  uint32_t raw = encoder.readRawAngle21();

  // 2) Read CRC and status bits
  uint8_t status = encoder.getStatus();
  bool crcErr = (status & MT6835_CRC_ERROR);

  // 3) Convert to floating-point radians (with CRC check)
  float angleRad = encoder.getCurrentAngle();

  // 4) Print results
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

  // 5) Every 5 seconds, toggle ABZ enable and report
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
