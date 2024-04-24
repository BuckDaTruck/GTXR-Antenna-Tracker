
#include <CNCShield.h>

#define NO_OF_STEPS 6000
#define SLEEP_BETWEEN_STEPS_MS 3
#define SPEED_STEPS_PER_SECOND 475
const int buttonTilt = 9;  // Pin for tilt limit switch
const int buttonPan = 10;  // Pin for pan limit switch
bool stopTilt = false;
bool stopPan = false;
int tiltState = 1;
int PanState = 1;
CNCShield cnc_shield;
StepperMotor *motorTilt = cnc_shield.get_motor(0);
StepperMotor *motorPan = cnc_shield.get_motor(1);

int currentPanPosition = 0;
int currentTiltPosition = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("I am Alive");
  pinMode(buttonPan, INPUT_PULLUP);
  pinMode(buttonTilt, INPUT_PULLUP);

  cnc_shield.begin();
  cnc_shield.enable();
  home();
}

void home() {
   motorPan->set_dir(COUNTER);
  motorTilt->set_dir(COUNTER);
  motorPan->set_speed(SPEED_STEPS_PER_SECOND);
  motorTilt->set_speed(SPEED_STEPS_PER_SECOND);
  motorPan->step(125);
  motorTilt->step(125);

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

  while (stopTilt == false) {
    int tiltState = digitalRead(buttonTilt);
    motorTilt->step();
    delay(SLEEP_BETWEEN_STEPS_MS);
    if (tiltState == LOW) {
      stopTilt = true;
    }
  }
  stopTilt = false;
  stopPan = false;
  tiltState = 1;
  PanState = 1;
  // Reset positions
  currentPanPosition = 0;
  currentTiltPosition = 0;
  moveMotorsToAngle(10, 15);
}
void loop() {
  if (Serial.available()) {  //example command for this sentax "M3 175" 3 degree pan 175 tilt
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("M")) {
      Serial.println("Moving!");
      int separatorIndex = command.indexOf(' ');
      int panAngle = command.substring(1, separatorIndex).toInt();
      int tiltAngle = command.substring(separatorIndex + 1).toInt();
      //Serial.println("Angles Pan" + panAngle.toString() + "Tilt" + tiltAngle.toString());
      moveMotorsToAngle(panAngle, tiltAngle);
    } else if (command.startsWith("H")) {
      Serial.println("homing!");
      home();
    }
  }
  /*if (Serial.available() > 0) {// this is for the python script
  
    char command = Serial.read();
    if (command == 'H') {
      home();
    } else if (command == 'M') {

      if (Serial.available() >= 8) {
        int panAngle, tiltAngle;
        Serial.readBytes((char *)&panAngle, sizeof(panAngle));
        Serial.readBytes((char *)&tiltAngle, sizeof(tiltAngle));
        moveMotorsToAngle(panAngle, tiltAngle);
      }
      
  }*/
}
void moveMotorsToAngle(int panAngle, int tiltAngle) {
  // Calculate steps from angles
  if (panAngle > 340) {
    panAngle = 340;
  }
  if (tiltAngle > 90) {
    tiltAngle = 90;
  }
  
  int targetPanSteps = map(panAngle, 0, 360, 0, NO_OF_STEPS);
  int targetTiltSteps = map(tiltAngle, 0, 360, 0, NO_OF_STEPS);
  if (currentPanPosition < targetPanSteps) {
    motorPan->set_dir(COUNTER);
  } else if (currentPanPosition > targetPanSteps) {
    motorPan->set_dir(CLOCKWISE);
  }
  if (currentTiltPosition < targetTiltSteps) {
    motorTilt->set_dir(COUNTER);
  } else if (currentTiltPosition > targetTiltSteps) {
    motorTilt->set_dir(CLOCKWISE);
  }

  // Calculate difference from current position
  int panStepDiff = abs(targetPanSteps - currentPanPosition);
  int tiltStepDiff = abs(targetTiltSteps - currentTiltPosition);

  // Move motors
  motorPan->set_speed(SPEED_STEPS_PER_SECOND);
  motorTilt->set_speed(SPEED_STEPS_PER_SECOND);
  motorPan->step(panStepDiff);
  motorTilt->step(tiltStepDiff);

  // Update current positions
  currentPanPosition = targetPanSteps;
  currentTiltPosition = targetTiltSteps;
}
