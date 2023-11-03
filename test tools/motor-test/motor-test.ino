#include <CNCShield.h>

#define NO_OF_STEPS 800
#define SLEEP_BETWEEN_STEPS_MS 0
#define SPEED_STEPS_PER_SECOND 400

/*
 * Create a CNCShield object and get a pointer to motor 0 (X axis).
   - motor_id 0 -> X axis
    - motor_id 1 -> Y axis
  - motor_id 2 -> Z axis
 */
CNCShield cnc_shield;
StepperMotor *motor = cnc_shield.get_motor(0);

void setup() {
  /*
   * Calling CNCShield.begin() is mandatory before using any motor.
   */
  cnc_shield.begin();

  /*
   * Enable the shield (set enable pin to LOW).
   */
  cnc_shield.enable();

  /*
   * Step in a previously set direction.
   */
  motor->set_dir(CLOCKWISE);
  for (int i = 0; i < NO_OF_STEPS; i++) {
    motor->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
  }

  motor->set_dir(COUNTER);
  for (int i = 0; i < NO_OF_STEPS; i++) {
    motor->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
  }

  /*
   * Step in a direction.
   */
  for (int i = 0; i < NO_OF_STEPS; i++) {
    motor->step(CLOCKWISE);
    delay(SLEEP_BETWEEN_STEPS_MS);
  }

  for (int i = 0; i < NO_OF_STEPS; i++) {
    motor->step(COUNTER);
    delay(SLEEP_BETWEEN_STEPS_MS);
  }

  /*
   *  Step a number of steps in a previously set direction
   *   with a previously set speed.
   */
  motor->set_speed(SPEED_STEPS_PER_SECOND);
  motor->set_dir(CLOCKWISE);
  motor->step(800);

  motor->set_dir(COUNTER);
  motor->step(800);

  /*
   *  Step a number of steps in a direction
   *   with a previously set speed.
   */
  motor->set_speed(SPEED_STEPS_PER_SECOND);
  motor->step(800, CLOCKWISE);
  motor->step(800, COUNTER);

  /*
   * Disable the shield (set enable pin to HIGH).
   */
  cnc_shield.disable();
}

void loop() {
  // put your main code here, to run repeatedly:
}
