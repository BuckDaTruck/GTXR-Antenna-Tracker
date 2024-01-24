#include <CNCShield.h>

#define NO_OF_STEPS 8000
#define SLEEP_BETWEEN_STEPS_MS 1.7
#define SPEED_STEPS_PER_SECOND 300
const int buttonTilt = 9;
int tiltState = 1;
const int buttonPan = 10;
int PanState = 1;
/*
 * Create a CNCShield object and get a pointer to motor 0 (X axis).
   - motor_id 0 -> X axis
    - motor_id 1 -> Y axis
  - motor_id 2 -> Z axis
 */
CNCShield cnc_shield;
StepperMotor *motorTilt = cnc_shield.get_motor(0);
StepperMotor *motorPan = cnc_shield.get_motor(1);
bool stopTilt = false;
bool stopPan = false;
void setup() {
  pinMode(buttonPan, INPUT);
  pinMode(buttonTilt, INPUT);
  /*
   * Calling CNCShield.begin() is mandatory before using any motor.
   */
  cnc_shield.begin();

  /*
   * Enable the shield (set enable pin to LOW).
   */
  cnc_shield.enable();
  home();  //home all axis
  /*
   * Disable the shield (set enable pin to HIGH).
   */
  //cnc_shield.disable();
}
void home() {
  motorPan->set_dir(CLOCKWISE);
  motorTilt->set_dir(CLOCKWISE);

  while (stopPan == false) {
    int panState = digitalRead(buttonPan);
    motorPan->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
    if (panState == LOW) {
      stopPan = true;
    }
  }
  motorPan->set_speed(SPEED_STEPS_PER_SECOND);
  motorPan->set_dir(COUNTER);
  motorPan->step(800);
  while (stopTilt == false) {
    int tiltState = digitalRead(buttonTilt);
    motorTilt->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
    if (tiltState == LOW) {
      stopTilt = true;
    }
  }

  motorTilt->set_speed(SPEED_STEPS_PER_SECOND);
  motorTilt->set_dir(COUNTER);
  motorTilt->step(800);
}
void loop() {
  motorPan->set_speed(SPEED_STEPS_PER_SECOND);
  motorPan->step(1400, COUNTER);
  motorPan->step(1400, CLOCKWISE);
  motorTilt->set_speed(SPEED_STEPS_PER_SECOND);
  motorTilt->step(1400, COUNTER);
  motorTilt->step(1400, CLOCKWISE);
}
