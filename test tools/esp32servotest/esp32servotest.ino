#include <ESP32Servo.h>



Servo servo1;
Servo servo2;

int servo1Pin = 32;
int servo2Pin = 33;

void setup() {
  Serial.begin(115200);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(0);
  servo2.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
}


void loop() {
  if (Serial.available() >= 8) {
    int degrees1;
    int degrees2;

    Serial.readBytes((char *)&degrees1, sizeof(degrees1));
    Serial.readBytes((char *)&degrees2, sizeof(degrees2));

    // Set the servo positions in degrees
    servo1.write(degrees1);
    servo2.write(degrees2);

    // Print the servo positions to the serial monitor
    Serial.print("Servo 1 Position (degrees): ");
    Serial.println(degrees1);
    Serial.print("Servo 2 Position (degrees): ");
    Serial.println(degrees2);
  }
}
