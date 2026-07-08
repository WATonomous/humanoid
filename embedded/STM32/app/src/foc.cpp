
/** For setup
 * - initialize PWM
 * - initialize angle/position encoder (MT6835)
 * - intialize current sensing
 * - intialize motor driver
 * - intialize motor configurations (PID, mode) + motor.initFOC()
 * - intialize CANFD and enable RX ISR
 *
 */
void setup() {}

/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
 *
 */
void loop() {}