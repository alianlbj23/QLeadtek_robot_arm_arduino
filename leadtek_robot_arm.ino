/***************************************************
  Motor Test - IIC Motor Drive (RZ7889 x 4)
  Servo Control

  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library
  motor driver iic Introduction: http://www.yfrobot.com.cn/wiki/index.php?title=MotorDriver_IIC
  motor driver iic：https://item.taobao.com/item.htm?id=626324653253

  YFROBOT ZL
  08/13/2020
 ****************************************************/
#include <Servo.h>
#include <ArduinoJson.h>
#include "MotorDriver.h"

#define MOTORTYPE YF_IIC_RZ   // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  servo.attach(10);
  delay(1000);   // wait 2s
  Serial.println("Start...");
}

int angle = 0;

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
    motorDriver.servoWrite(S1, servoAngles[0]);
    motorDriver.servoWrite(S2, servoAngles[1]);
    motorDriver.servoWrite(S3, servoAngles[2]);
    motorDriver.servoWrite(S4, servoAngles[3]);
    motorDriver.servoWrite(S5, servoAngles[4]);
    motorDriver.servoWrite(10, servoAngles[5]);
    // For the additional servo not controlled by motor driver
    // servo.write(servoAngles[4]);
    servo.write(0);
  }
}
