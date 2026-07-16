// Gives access to Arduino functions like Serial, delay(), and pin names.
#include <Arduino.h>

// Gives access to the STM32 SPI peripheral through the Arduino SPI API.
#include <SPI.h>

// Gives access to SimpleFOC base classes like BLDCMotor.
#include <SimpleFOC.h>

// Gives access to the SimpleFOCDrivers MT6835 SPI encoder driver.
#include <encoders/mt6835/MagneticSensorMT6835.h>

// SPI clock pin connected from the STM32 to the MT6835 SCK pin.
#define MT6835_SCK PA5

// SPI MISO pin connected from the MT6835 data output to the STM32.
#define MT6835_MISO PA6

// SPI MOSI pin connected from the STM32 data output to the MT6835.
#define MT6835_MOSI PA7

// Chip-select pin used to choose the MT6835 during SPI communication.
#define MT6835_CS PA0

// MT6835 SPI configuration: 1 MHz, most-significant bit first, SPI mode 3.
SPISettings mt6835SPISettings = SPISettings(1000000, MSBFIRST, SPI_MODE3);

// SimpleFOC sensor object that knows how to read angle data from the MT6835.
MagneticSensorMT6835 encoder = MagneticSensorMT6835(MT6835_CS, mt6835SPISettings);

// SimpleFOC motor object. The value 7 is the motor pole-pair count.
BLDCMotor motor = BLDCMotor(7);

static void printEncoderStatus(uint8_t status)
{
  if (status == 0)
  {
    Serial.print("OK");
    return;
  }

  if (status & MT6835_STATUS_OVERSPEED)
    Serial.print("OVERSPEED ");
  if (status & MT6835_STATUS_WEAKFIELD)
    Serial.print("WEAK_FIELD ");
  if (status & MT6835_STATUS_UNDERVOLT)
    Serial.print("UNDERVOLT ");
  if (status & MT6835_CRC_ERROR)
    Serial.print("CRC_ERROR ");
}

static void printEncoderSample()
{
  const uint32_t rawAngle = encoder.readRawAngle21();

  // update() lets SimpleFOC refresh its cached angle and apply CRC checking.
  encoder.update();

  const float angleRad = encoder.getAngle();
  const uint8_t status = encoder.getStatus();

  Serial.print("angle_rad=");
  Serial.print(angleRad, 6);
  Serial.print(" raw=");
  Serial.print(rawAngle);
  Serial.print(" status=");
  printEncoderStatus(status);
  Serial.println();
}

/** For setup 
 * - initialize PWM
 * - initialize angle/position encoder (MT6835)
 * - intialize current sensing
 * - intialize motor driver
 * - intialize motor configurations (PID, mode) + motor.initFOC()
 * - intialize CANFD and enable RX ISR
 *
*/
void setup()
{
  // Start USB serial at 115200 baud so angle readings can be printed.
  Serial.begin(115200);

  // Do not wait forever here: some STM32 serial paths never report "ready".
  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs < 2000))
    delay(10);

  Serial.println("MT6835 encoder bring-up");

  // Assign the STM32 SPI pins that are wired to the MT6835 before SPI begins.
  SPI.setSCLK(MT6835_SCK);
  SPI.setMISO(MT6835_MISO);
  SPI.setMOSI(MT6835_MOSI);

  // Keep chip-select idle high so the encoder is not selected during boot.
  pinMode(MT6835_CS, OUTPUT);
  digitalWrite(MT6835_CS, HIGH);

  // Enable CRC checking so bad SPI frames are visible in the status output.
  encoder.checkcrc = true;

  // Initialize the MT6835 driver; this also starts SPI communication.
  encoder.init();

  // Connect the encoder to SimpleFOC so the motor can use it for angle feedback.
  motor.linkSensor(&encoder);

  // Print one startup sample. Rotate the shaft and the angle/raw values should move.
  printEncoderSample();
}


/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
 * 
 */
void loop()
{
  printEncoderSample();

  // Slow the serial output so the monitor is readable.
  delay(100);
}
