
<<<<<<< HEAD
/** For setup
=======
/** For setup 
>>>>>>> d0b14223 (FOC and ring buffer impl (#65))
 * - initialize PWM
 * - initialize angle/position encoder (MT6835)
 * - intialize current sensing
 * - intialize motor driver
 * - intialize motor configurations (PID, mode) + motor.initFOC()
 * - intialize CANFD and enable RX ISR
 *
<<<<<<< HEAD
 */
void setup() {}
=======
*/
void setup()
{



}

>>>>>>> d0b14223 (FOC and ring buffer impl (#65))

/** For loop
 *  -call loopFOC, move
 *  -check ringbuffer for messages
 *  -send telemetry data back
<<<<<<< HEAD
 *
 */
void loop() {}
=======
 * 
 */
void loop()
{


}
>>>>>>> d0b14223 (FOC and ring buffer impl (#65))
