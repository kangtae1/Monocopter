#include <Servo.h>

#define SERVO_ANGLE_MIN -45.0
#define SERVO_ANGLE_MAX 45.0
#define SERVO_RAW_MIN 1000
#define SERVO_RAW_MAX 2000
#define SERVO_1_ANGLE 30.0
#define SERVO_2_ANGLE 150.0
#define SERVO_3_ANGLE 270.0

Servo servo1;
Servo servo2;
Servo servo3;

double target_angle;
double target_power;

double servo1_angle;
double servo2_angle;
double servo3_angle;
double servo1_pos;
double servo2_pos;
double servo3_pos;

void setup() {
  servo1.attach(9, SERVO_RAW_MIN, SERVO_RAW_MAX);
  servo2.attach(10, SERVO_RAW_MIN, SERVO_RAW_MAX);
  servo3.attach(11, SERVO_RAW_MIN, SERVO_RAW_MAX);

  Serial.begin(9600);
}

void loop() {
  // 목표 각도 지정
  target_angle = random(0, 360);
  target_power = random(0, 100) * 0.01;

  // 각 서보 모터의 각도를 지정
  servo1_angle = 45.0 * sin(radians(target_angle - SERVO_1_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;
  servo2_angle = 45.0 * sin(radians(target_angle - SERVO_2_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;
  servo3_angle = 45.0 * sin(radians(target_angle - SERVO_3_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;

  // 동작에 필요한 펄스 시간 계산
  servo1_pos = servo1_angle * target_power * (500.0 / 45.0) + 1500.0;
  servo2_pos = servo2_angle * target_power * (500.0 / 45.0) + 1500.0;
  servo3_pos = servo3_angle * target_power * (500.0 / 45.0) + 1500.0;

  // 각 서보 모터에 입력 값을 적용
  servo1.writeMicroseconds(servo1_pos);
  servo2.writeMicroseconds(servo2_pos);
  servo3.writeMicroseconds(servo3_pos);

  Serial.println();
  Serial.println(servo1_angle);
  Serial.println(servo2_angle);
  Serial.println(servo3_angle);

  delay(1000);
}
