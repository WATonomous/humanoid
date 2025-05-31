#include <Arduino.h>
#include <SimpleFOC.h>

//--- SPI pin assignments on ESP32 VSPI (defaults) ---
//  MOSI = GPIO 23
//  MISO = GPIO 19
//  SCK  = GPIO 18
//  (CS can be any free GPIO; we chose 10)
constexpr int CS_PIN = 10;

//--- Create a 21‑bit MT6835 sensor on SPI ---
// Option A: use default angleRegister=0
MagneticSensorSPI sensor(CS_PIN, 21);
// (If you want to force reading register 0x003, use: MagneticSensorSPI sensor(CS_PIN, 21, 0x003);)

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait for Serial

  Serial.println("SimpleFOC MT6835 SPI Test");

  //--- Initialize sensor (this does SPI.begin() + pinMode(CS_PIN)) ---
  sensor.init();

  //--- Read and print one initial angle so we know it's working ---
  sensor.update();
  float a0 = sensor.getAngle();  // returns radians in [0 .. 2π)
  Serial.print("Startup angle (rad): ");
  Serial.println(a0, 6);
  Serial.print("Startup angle (deg): ");
  Serial.println(a0 * 180.0 / M_PI, 4);

  Serial.println("Ready. Reading angle every 50ms...\n");
}

void loop() {
  //--- 1) Trigger an SPI read of the MT6835 (21‑bit raw → [0..2π) float) ---
  sensor.update();

  //--- 2) Fetch the angle in radians, and convert to degrees ---
  float angleRad = sensor.getAngle();
  float angleDeg = angleRad * 180.0 / M_PI;

  //--- 3) Print it out ---
  Serial.print("Angle = ");
  Serial.print(angleRad, 6);
  Serial.print("rad (");
  Serial.print(angleDeg, 2);
  Serial.println("°)");

  //--- 4) Wait 50 ms before next read (~20 Hz) ---
  delay(50);
}
