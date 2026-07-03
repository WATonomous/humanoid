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
  // initialize angle/position encoder (MT6835)

  // Start USB serial at 115200 baud so angle readings can be printed.
  Serial.begin(115200);

  // Wait until the serial port is open before printing test output.
  while (!Serial)
    ;

  // Assign the STM32 SPI pins that are wired to the MT6835.
  SPI.setSCLK(MT6835_SCK);
  SPI.setMISO(MT6835_MISO);
  SPI.setMOSI(MT6835_MOSI);

  // Initialize the MT6835 driver; this also starts SPI communication.
  encoder.init();

  // Read the first encoder sample so getAngle() has a fresh value.
  encoder.update();

  // Connect the encoder to SimpleFOC so the motor can use it for angle feedback.
  motor.linkSensor(&encoder);

  // Print the initial measured shaft angle in radians.
  Serial.println(encoder.getAngle());

}


/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
 * 
 */
void loop()
{
  // Read the latest angle from the MT6835 over SPI.
  encoder.update();

  // Print the current angle in radians; rotate the shaft and this should change.
  Serial.println(encoder.getAngle());

  // Slow the serial output so the monitor is readable.
  delay(100);
}
