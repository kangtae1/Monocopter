#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(9, 1000, 2000);
  Serial.begin(9600);
}

int angle;

void loop() {
  angle = 0;
  myservo.write(map(angle, -45, 45, 0, 168));
  Serial.println(map(angle, -45, 45, 0, 168));
  delay(500);
}
