#include <Servo.h>
#include <ArduinoJson.h>
#include "MotorDriver.h"
#define CUSTOM_ID "ARM001"
#define MOTORTYPE YF_IIC_RZ   // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;
#define HomePosIO  11
#define LED 13
#define PowerEnable 12

int Heading = 90;
int NextHeading = 90;
int Ang_34p = 175;
// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();
void drv_S3_S4( int angS34)
{
 motorDriver.servoWrite(S3, angS34); 
 motorDriver.servoWrite(S4, angS34); 
}

void drv_s34(int NextHead, int delay_drv)
{
  while( NextHead != Ang_34p)
  {  
    if( NextHead > Ang_34p) Ang_34p = Ang_34p +1;
    else Ang_34p = Ang_34p - 1;
    motorDriver.servoWrite(S3, Ang_34p); 
    motorDriver.servoWrite(S4, 90 + (180 -Ang_34p)); 
    delay(delay_drv);    

    Serial.print("Target = ");
    Serial.print(NextHead);    
    Serial.print("Ang_34p = ");
    Serial.println(Ang_34p);    
  }  
}

void drv_s1(int NextHead, int delay_s1)
{
  while( NextHead != Heading)
  {  
    if( NextHead > Heading) Heading = Heading +1;
    else Heading = Heading - 1;
    motorDriver.servoWrite(S1, Heading); 
    delay(delay_s1);    
  }  
}


int currentAngles[6] = {90, 90, 90, 90, 90, 90}; // S1, S2, S3, S4, S5, and the additional servo

void setup() {
  Serial.begin(115200);
  Serial.println(CUSTOM_ID);
  int count = 0;
  
  Serial.println("Motor Drive test!");

  pinMode( HomePosIO, INPUT_PULLUP);
  pinMode( LED, OUTPUT);
  pinMode( PowerEnable, OUTPUT);
  digitalWrite( PowerEnable, LOW);

  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  servo.attach(10);
  // Initialize servos to the initial positions
  
  // motorDriver.servoWrite(S1, currentAngles[0]);
  // motorDriver.servoWrite(S2, currentAngles[1]);
  // motorDriver.servoWrite(S3, currentAngles[2]);
  // motorDriver.servoWrite(S4, currentAngles[3]);
  // motorDriver.servoWrite(S5, currentAngles[4]);
  // servo.write(currentAngles[5]);

  int  Home = digitalRead( HomePosIO) ;  
  #if 1
    while( Home == HIGH)
    {
      if(++count < 10)   
          digitalWrite( LED, LOW);
      else
      {
        digitalWrite( LED, HIGH);
        count = 0;
      }
      delay(50);  
      Home = digitalRead( HomePosIO) ;
    }
  #endif 
  digitalWrite( LED, LOW);
  delay(250);
  digitalWrite( LED, HIGH);
  delay(3000);  
  digitalWrite( PowerEnable, HIGH);
  for( int i =0 ; i<10;i++)
  {
    delay(100);
    if(  digitalRead( HomePosIO) == HIGH )
      {
        digitalWrite( LED, LOW);
      }
  }    
  digitalWrite( LED, LOW);


  // int targetAngleS1 = 90;
  // int initialAngleS2 = 20;
  // int targetAngleS3 = 90;
  // int targetAngleS4 = 90;
  // int targetAngleS5 = 90;
  
  int t1 = 20;
  int delay_time = 500;
  int angle = 0;
  angle = 160;
  // moveServoGradually(S1, 0, delay_time, targetAngleS1);
  motorDriver.servoWrite(S1, 90); 
  motorDriver.servoWrite(S2, 90);
  motorDriver.servoWrite(S3, 90);
  motorDriver.servoWrite(S4, 90);
  motorDriver.servoWrite(S5, 90);
  servo.write(90);
  // moveServoGradually(S2, 90, 80, initialAngleS2);
  // // motorDriver.servoWrite(S2, 90);
  // while( angle > 90) 
  // {
    
  //   // motorDriver.servoWrite(S2, 90); 
  //   // motorDriver.servoWrite(S3, 90); 
  //   moveServoGradually(S3, 0, delay_time, targetAngleS3);
    
  //   delay(t1);  
  //   angle = angle - 1;
  // }
  // moveServoGradually(S3, 0, delay_time, targetAngleS3); 
  // delay(t1); 

  // motorDriver.servoWrite(S4, 90); 
  // // moveServoGradually(S4, 0, delay_time, targetAngleS4);
  // motorDriver.servoWrite(S5, 90);
  // // moveServoGradually(S5, 0, delay_time, targetAngleS5); 

  delay(1000);   // wait 2s
  // Serial.println("Start...");
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
