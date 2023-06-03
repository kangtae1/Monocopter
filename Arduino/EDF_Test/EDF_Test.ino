#include <Servo.h>

Servo motor;
int motorVal = 0; // 실제 모터에 입력되는 PWM duty %
int inputVal = 0; // Throttle 입력 %

void setup() {
  motor.attach(9, 1000, 2000);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    inputVal = Serial.parseInt();
    if (inputVal >= 0 && inputVal <= 100) {
      motorVal = map(inputVal, 0, 100, 0, 180); // Throttle 8% 이상에서 회전 시작
    }
    Serial.read();
  }

  motor.write(motorVal);
  Serial.print("Throttle: ");
  Serial.print(inputVal);
  Serial.print("%, Motor Value: ");
  Serial.println(motorVal);

  delay(100);
}
