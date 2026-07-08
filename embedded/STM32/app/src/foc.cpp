
/** For setup 
 * - initialize PWM
 * - initialize angle/position encoder (MT6835)
 * - intialize motor driver
 * - intialize motor configurations (PID, mode) + motor.initFOC()
 *
*/

#include <SimpleFOC.h>
#include <encoders/mt6835/MagneticSensorMT6835.h>

BLDCMotor motor = BLDCMotor(7);

// enable pin might need to be changed
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_8, PA_9, PA_10, PB_5);
SPISettings encoderSettings = SPISettings(1e6, MSBFIRST, SPI_MODE3);
MagneticSensorMT6835 encoder = MagneticSensorMT6835(PA_0, encoderSettings);
float target_voltage = 0.0f;

void setup()
{

  Serial.begin(115200);

  SimpleFOCDebug::enable(&Serial);
  
  // need to check with motor driver specs
  driver.voltage_power_supply = 12.0;
  // used for initial testing
  driver.voltage_limit = 2.0;
  // might need to be changed
  driver.pwm_frequency = 20000;

  encoder.init();

  if (!driver.init()){
    Serial.println("Driver init failed!");
    while (1) {}
  }

  driver.enable();
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  motor.voltage_limit = 2.0;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  if (!motor.init()) {
    Serial.println("Motor init failed!");
    while (1) {}
  }
  motor.initFOC();

  Serial.println("PWM, driver, and motor initialized.");
  delay(1000);

}


/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
 * 
 */
void loop()
{
  motor.loopFOC();
  motor.move(target_voltage);
}
