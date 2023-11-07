#include <CNCShield.h>
#define NO_OF_STEPS 800
#define SLEEP_BETWEEN_STEPS_MS 0
#define SPEED_STEPS_PER_SECOND 400
#define LIMIT_SWITCH_X 9
#define LIMIT_SWITCH_Y 10
#define GEAR_RATIO 10.0 / 36.0
CNCShield cnc_shield;
StepperMotor *motor_x = cnc_shield.get_motor(0);
StepperMotor *motor_y = cnc_shield.get_motor(1);
const float X_MAX_ANGLE_DEG = 90.0;
const float Y_MAX_ANGLE_DEG = 345.0;
void setup() {
  cnc_shield.begin();
  cnc_shield.enable();
  motor_x->set_dir(COUNTER);
  while (digitalRead(LIMIT_SWITCH_X) == LOW) {
    motor_x->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
  }
  motor_x->set_speed(SPEED_STEPS_PER_SECOND);
  motor_x->step(45);
  motor_y->set_dir(COUNTER);
  while (digitalRead(LIMIT_SWITCH_Y) == LOW) {
    motor_y->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
  }
  motor_y->set_speed(SPEED_STEPS_PER_SECOND);
  motor_y->step(45);
  cnc_shield.disable();
}
void loop() {
  cnc_shield.enable();
  float current_angle_x = 45.0; 
  while (current_angle_x < X_MAX_ANGLE_DEG && digitalRead(LIMIT_SWITCH_X) == HIGH) {
    int steps = int(1.0 / GEAR_RATIO * NO_OF_STEPS);
    motor_x->set_speed(SPEED_STEPS_PER_SECOND);
    motor_x->step(steps, CLOCKWISE);
    delay(SLEEP_BETWEEN_STEPS_MS);
    current_angle_x += 1.0;
  }
  float current_angle_y = 45.0;  
  while (current_angle_y < Y_MAX_ANGLE_DEG && digitalRead(LIMIT_SWITCH_Y) == HIGH) {
    int steps = int(1.0 / GEAR_RATIO * NO_OF_STEPS);
    motor_y->set_speed(SPEED_STEPS_PER_SECOND);
    motor_y->step(steps, CLOCKWISE);
    delay(SLEEP_BETWEEN_STEPS_MS);
    current_angle_y += 1.0;
  }
  cnc_shield.disable();
}
