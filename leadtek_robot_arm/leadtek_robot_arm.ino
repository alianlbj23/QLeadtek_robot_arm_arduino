#include <Servo.h>
#include <ArduinoJson.h>
#include "MotorDriver.h"
#define CUSTOM_ID "usb_robot_arm"
#define MOTORTYPE YF_IIC_RZ
const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;
#define HomePosIO  11
#define bee 13
#define PowerEnable 12
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();
int currentAngles[6] = {90, 90, 90, 90, 90, 90}; // S1, S2, S3, S4, S5, and the additional servo
unsigned long lastInitialSettingTime = 0;
int initialSettingState = 0;
bool initialSettingComplete = false;

void setup() {
  Serial.begin(115200);  
  pinMode( HomePosIO, INPUT_PULLUP);
  pinMode( bee, OUTPUT);
  pinMode( PowerEnable, OUTPUT);
  digitalWrite( PowerEnable, LOW);
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50);
  servo.attach(10);
  
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
  if (!initialSettingComplete) {
    digitalWrite(PowerEnable, LOW);
    performInitialSetting();
  }

  if (Serial.available() > 0) {
    String jsonString = Serial.readString();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (!error) {
      if (doc.containsKey("command") && doc["command"] == "I") {
        Serial.println(CUSTOM_ID);
      } 
      if(initialSettingComplete == true){
        JsonArray servoAngles = doc["servo_target_angles"];
        moveServoGradually(S1, servoAngles[0], 50, currentAngles[0]);
        moveServoGradually(S2, servoAngles[1], 50, currentAngles[1]);
        moveServoGradually(S3, servoAngles[2], 50, currentAngles[2]);
        moveServoGradually(S4, servoAngles[3], 50, currentAngles[3]);
        moveServoGradually(S5, servoAngles[4], 50, currentAngles[4]);
        moveServoGradually(servo, servoAngles[5], 50, currentAngles[5]);
      } 
    }
  }
}
void performInitialSetting() {
  unsigned long currentTime = millis();
  
  switch (initialSettingState) {
    case 0:
      if (digitalRead(HomePosIO) == LOW) {
        initialSettingState = 1;
        lastInitialSettingTime = currentTime;
      } else {
        digitalWrite(bee, (currentTime / 500) % 2);
      }
      break;
      
    case 1:
      digitalWrite(bee, LOW);
      if (currentTime - lastInitialSettingTime >= 250) {
        initialSettingState = 2;
        lastInitialSettingTime = currentTime;
      }
      break;
      
    case 2:
      digitalWrite(bee, HIGH);
      if (currentTime - lastInitialSettingTime >= 3000) {
        initialSettingState = 3;
      }
      break;
      
    case 3:
      digitalWrite(PowerEnable, HIGH);
      digitalWrite(bee, LOW);
      motorDriver.servoWrite(S1, 90);
      motorDriver.servoWrite(S2, 90);
      motorDriver.servoWrite(S3, 90);
      motorDriver.servoWrite(S4, 90);
      motorDriver.servoWrite(S5, 90);
      servo.write(90);
      initialSettingComplete = true;
      break;
  }
}
