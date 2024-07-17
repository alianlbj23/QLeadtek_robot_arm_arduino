#include <Servo.h>
#include <ArduinoJson.h>
#include "MotorDriver.h"

#define MOTORTYPE YF_IIC_RZ   // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();

int currentAngles[6] = {90, 100, 20, 80, 30, 0}; // S1, S2, S3, S4, S5, and the additional servo

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  servo.attach(10);

  // Initialize servos to the initial positions
  motorDriver.servoWrite(S1, currentAngles[0]);
  motorDriver.servoWrite(S2, currentAngles[1]);
  motorDriver.servoWrite(S3, currentAngles[2]);
  motorDriver.servoWrite(S4, currentAngles[3]);
  motorDriver.servoWrite(S5, currentAngles[4]);
  servo.write(currentAngles[5]);

  delay(1000);   // wait 2s
  Serial.println("Start...");
}

void moveServoGradually(int servoIndex, int targetAngle, int stepDelay, int &currentAngle) {
  if (targetAngle == -1) return; // Don't move if the target angle is -1
  int step = (currentAngle < targetAngle) ? 1 : -1;
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    motorDriver.servoWrite(servoIndex, angle);
    delay(stepDelay);
  }
  motorDriver.servoWrite(servoIndex, targetAngle);
  currentAngle = targetAngle;
}

void moveServoGradually(Servo &servo, int targetAngle, int stepDelay, int &currentAngle) {
  if (targetAngle == -1) return; // Don't move if the target angle is -1
  int step = (currentAngle < targetAngle) ? 1 : -1;
  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    servo.write(angle);
    delay(stepDelay);
  }
  servo.write(targetAngle);
  currentAngle = targetAngle;
}

void loop() {
  if (Serial.available() > 0) {
    String jsonString = Serial.readString();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    JsonArray servoAngles = doc["servo_target_angles"];
    moveServoGradually(S1, servoAngles[0], 50, currentAngles[0]);
    moveServoGradually(S2, servoAngles[1], 50, currentAngles[1]);
    moveServoGradually(S3, servoAngles[2], 50, currentAngles[2]);
    moveServoGradually(S4, servoAngles[3], 50, currentAngles[3]);
    moveServoGradually(S5, servoAngles[4], 50, currentAngles[4]);
    moveServoGradually(servo, servoAngles[5], 50, currentAngles[5]);
  }
}
