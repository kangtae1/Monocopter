#include <Servo.h>
#include <Bluepad32.h>

#define LEDR 27
#define LEDG 25
#define LEDB 26

#define SERVO_ANGLE_MIN -50.0
#define SERVO_ANGLE_MAX 50.0
#define SERVO_RAW_MIN 900
#define SERVO_RAW_MAX 2100
#define SERVO_RAW_CENTER 1500
#define SERVO_1_ANGLE 30.0
#define SERVO_2_ANGLE 150.0
#define SERVO_3_ANGLE 270.0
#define TARGET_POWER_FACTOR 336400

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

Servo EDF;
Servo Vane1;
Servo Vane2;
Servo Vane3;

int throttle;
int pitch;
int roll;
int yaw;
int motorVal;
int servo1Angle;
int servo2Angle;
int servo3Angle;
int servo1Val;
int servo2Val;
int servo3Val;
double target_angle;
double target_power;

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial
  Serial.begin(2000000);
  // while (!Serial) {
  //   // wait for serial port to connect.
  //   ;
  // }
  delay(500);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  // This call is mandatory. It setups Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }
  
  EDF.attach(9, SERVO_RAW_MIN, SERVO_RAW_MAX);
  Vane1.attach(6, SERVO_RAW_MIN, SERVO_RAW_MAX);
  Vane2.attach(4, SERVO_RAW_MIN, SERVO_RAW_MAX);
  Vane3.attach(5, SERVO_RAW_MIN, SERVO_RAW_MAX);
  EDF.writeMicroseconds(1000);
  Vane1.writeMicroseconds(SERVO_RAW_CENTER);
  Vane2.writeMicroseconds(SERVO_RAW_CENTER);
  Vane3.writeMicroseconds(SERVO_RAW_CENTER);

  BP32.pinMode(LEDR, OUTPUT);
  BP32.pinMode(LEDG, OUTPUT);
  BP32.pinMode(LEDB, OUTPUT);
  BP32.digitalWrite(LEDR, HIGH);
  BP32.digitalWrite(LEDG, HIGH);
  BP32.digitalWrite(LEDB, HIGH);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  digitalWrite(LED_BUILTIN, HIGH);
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);

      // Attatch EDF and Servos
      // if (!EDF.attached()) {
      //   digitalWrite(LED_BUILTIN, HIGH);
      //   EDF.attach(9, SERVO_RAW_MIN, SERVO_RAW_MAX);
      //   Vane1.attach(3, SERVO_RAW_MIN, SERVO_RAW_MAX);
      //   Vane2.attach(4, SERVO_RAW_MIN, SERVO_RAW_MAX);
      //   Vane3.attach(5, SERVO_RAW_MIN, SERVO_RAW_MAX);
      //   EDF.writeMicroseconds(1000);
      //   Vane1.writeMicroseconds(SERVO_RAW_CENTER);
      //   Vane2.writeMicroseconds(SERVO_RAW_CENTER);
      //   Vane3.writeMicroseconds(SERVO_RAW_CENTER);
      // }
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
      "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  digitalWrite(LED_BUILTIN, LOW);
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;

      // // detach EDF and Servos
      // if (EDF.attached()) {
      //   EDF.detach();
      //   Vane1.detach();
      //   Vane2.detach();
      //   Vane3.detach();
      // }
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
      "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr gamepad) {

  throttle = gamepad->throttle();
  pitch = gamepad->axisY();
  roll = gamepad->axisX();
  yaw = gamepad->axisRX();

  // Set EDF pulse time
  motorVal = (double)throttle * (1000.0 / 1024.0) + 1000.0;

  // Set control target angle and power
  target_angle = degrees(atan2(-pitch, roll));
  target_power = (double)(sq(pitch) + sq(roll)) / TARGET_POWER_FACTOR;

  // Calculate each servo's angle
  servo1Angle = 45.0 * sin(radians(target_angle - SERVO_1_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;
  servo2Angle = 45.0 * sin(radians(target_angle - SERVO_2_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;
  servo3Angle = 45.0 * sin(radians(target_angle - SERVO_3_ANGLE)), -1, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX;

  // Calculate servo pulse time
  servo1Val = servo1Angle * target_power * (500.0 / 50.0) + SERVO_RAW_CENTER;
  servo2Val = servo2Angle * target_power * (500.0 / 50.0) + SERVO_RAW_CENTER;
  servo3Val = servo3Angle * target_power * (500.0 / 50.0) + SERVO_RAW_CENTER;

  // Write to servo
  EDF.writeMicroseconds(motorVal);
  Vane1.writeMicroseconds(servo1Val);
  Vane2.writeMicroseconds(servo2Val);
  Vane3.writeMicroseconds(servo3Val);

  // Debug output
  char buf[200];
  snprintf(
    buf, sizeof(buf) - 1,
    "throttle: %4d, pitch: %4li, roll: %4li, yaw: %4li, target_angle: %4.2lf, target_power: %4.2lf, "
    "motorVal: %4d, servo1Val: %4d, servo2Val: %4d, servo3Val: %4d",
    throttle, pitch, roll, yaw, target_angle, target_power, motorVal, servo1Val, servo2Val, servo3Val);
  Serial.println(buf);
}

// Arduino loop function. Runs in CPU 1
void loop() {
  // Fetch all the controller info
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[0];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    }
  }
}
