
/** For setup 
 * - initialize PWM
 * - initialize angle/position encoder (MT6835)
 * - intialize current sensing
 * - intialize motor driver
 * - intialize motor configurations (PID, mode) + motor.initFOC()
 * - intialize CANFD and enable RX ISR
 *
*/

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);

// enable pin might need to be changed
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_8, PA_9, PA_10, PB_5);

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

  // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // enable driver
  driver.enable();
  motor.linkDriver(&driver);

  motor.voltage_limit = 2.0;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.init();

  Serial.println("PWM, driver, and motor initialized.");
  _delay(1000);

}


/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
 * 
 */
void loop()
{

}