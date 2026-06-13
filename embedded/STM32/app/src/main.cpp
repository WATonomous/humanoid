#include <SimpleFOC.h>

// SENSOR
Encoder sensor = Encoder(2, 3, 2048);
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}

// DRIVER
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// CURRENT SENSE (Optional)
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50, A0, A2);

// MOTOR
BLDCMotor motor = BLDCMotor(11);

// USER INTERFACE
Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);

  sensor.init();
  sensor.enableInterrupts(doA, doB);

  driver.voltage_power_supply = 12;
  driver.init();

  current_sense.linkDriver(&driver);
  current_sense.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  motor.init();
  motor.initFOC();

  commander.add('M', onMotor, "motor");
  Serial.println("Motor ready! Type 'M' for motor commands");
}

void loop() {
  motor.loopFOC();
  motor.move(motor.target);
  commander.run();
}