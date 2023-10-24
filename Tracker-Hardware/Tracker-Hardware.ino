#include <AccelStepper.h>

const int stepsPerRevolution = 720; // Number of steps per revolution for a 0.5-degree stepper
const int maxRotation = 320; // Maximum rotation from home (in degrees)

AccelStepper stepper1(AccelStepper::DRIVER, 2, 3); // Define stepper motor 1
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5); // Define stepper motor 2

const int switchPin1 = 6; // Pin for limit switch 1
const int switchPin2 = 7; // Pin for limit switch 2

void homeStepper(AccelStepper &stepper, int switchPin);

// Function prototype
void moveStepperWithinLimits(AccelStepper &stepper, float targetRotation, int maxRotation);

void setup() {
  stepper1.setMaxSpeed(1000); // Set the maximum speed for motor 1 (adjust as needed)
  stepper2.setMaxSpeed(1000); // Set the maximum speed for motor 2 (adjust as needed)

  pinMode(switchPin1, INPUT_PULLUP); // Set limit switch 1 as input with pull-up resistor
  pinMode(switchPin2, INPUT_PULLUP); // Set limit switch 2 as input with pull-up resistor

  Serial.begin(115200);

  // Set the speed in steps per second (adjust as needed)
  stepper1.setSpeed(100);
  stepper2.setSpeed(100);

  // Homing procedure upon startup
  homeStepper(stepper1, switchPin1);
  homeStepper(stepper2, switchPin2);
}

void homeStepper(AccelStepper &stepper, int switchPin) {
  // Move the stepper towards the limit switch while checking the switch state
  while (digitalRead(switchPin) == HIGH) {
    stepper.setSpeed(100); // Set a moderate speed when approaching the switch
    stepper.runSpeed();
  }
  stepper.setCurrentPosition(0); // Set the current position to zero

  // Move the stepper away from the switch slowly for accurate homing
  stepper.setSpeed(20); // Set a slower speed for fine homing
  while (stepper.currentPosition() != stepsPerRevolution) {
    stepper.runSpeed();
  }

  // Reset speed to the maximum speed
  stepper.setSpeed(stepper.maxSpeed());
}

void loop() {
  if (Serial.available() >= 8) {
    byte value1Bytes[sizeof(float)];
    byte value2Bytes[sizeof(float)];

    Serial.readBytes(value1Bytes, sizeof(float));
    Serial.readBytes(value2Bytes, sizeof(float));

    float value1, value2;
    memcpy(&value1, value1Bytes, sizeof(float));
    memcpy(&value2, value2Bytes, sizeof(float));

    String json = "{\"value1\":" + String(value1, 2) + ", \"value2\":" + String(value2, 2) + "}";

    // Ensure the motors don't exceed the defined limits
    moveStepperWithinLimits(stepper1, value1, maxRotation);
    moveStepperWithinLimits(stepper2, value2, maxRotation);
  }
}

void moveStepperWithinLimits(AccelStepper &stepper, float targetRotation, int maxRotation) {
  // Calculate the target position in steps
  int targetPosition = targetRotation * stepsPerRevolution / 360;

  // Ensure we don't exceed the defined limits
  targetPosition = constrain(targetPosition, 0, maxRotation * stepsPerRevolution / 360);

  // Move the stepper to the target position
  stepper.moveTo(targetPosition);

  // Run the stepper to the target position
  stepper.run();
}
