/* 
 * Set Board: Tools --> Board --> Arduino Mega2560
 * Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>
#include <AccelStepper.h>
#include <Ewma.h>
#include <ArduinoJson.h>
#include "configuration.h"


//=============== Function Name Declaration ===============
  double Deadband(double value,double limit);
  int OutputToMotor1(int value);
  int OutputToMotor2(int value);
  int OutputToMotor3(int value);
  int OutputToMotor4(int value);
  int limitThreshold(int value, int threshold);
  void moveForwardBackward(int value);
  void moveSideways(int value);
  void rotate(int value);
  void processGripperServo(int value, int threshold, Servo servo);
  void processArmServo(int value, Servo servo);
  

//=============== Parameters Declaration ==================

// Type Servo
  Servo myServo1;
  Servo myServo2;
  Servo myServo3;
  Servo myServo4;
// Time
  unsigned long previousLoopTime = 0;
  unsigned long loopTime = 0;
// Output Signal to Motor Driver  
  int out1 = 0;
  int out2 = 0;
  int out3 = 0;
  int out4 = 0;
// Processed Input Signal
  const int movement = 100;
// Json Declaration
  StaticJsonDocument<200> doc;


//===================== setup() ========================


void setup() {
// Set pinmode to read command signal from Test Switch.
  pinMode(buttonPin,INPUT);
// Set pinmode to write command signal to Motor Driver.
  pinMode(INA1,OUTPUT);
  pinMode(INB1,OUTPUT);
  pinMode(PWM1,OUTPUT);
  
  pinMode(INA2,OUTPUT);
  pinMode(INB2,OUTPUT);
  pinMode(PWM2,OUTPUT);

  pinMode(INA3,OUTPUT);
  pinMode(INB3,OUTPUT);
  pinMode(PWM3,OUTPUT);

  pinMode(INA4,OUTPUT);
  pinMode(INB4,OUTPUT);
  pinMode(PWM4,OUTPUT);

  // Set pinmode led
  pinMode(ledPin1,OUTPUT);
  pinMode(ledPin2,OUTPUT);
  pinMode(ledPin3,OUTPUT);
  pinMode(ledPin4,OUTPUT);

//===== Initialize Command =====
  // Initialize Motor Driver.
  digitalWrite(INA1,LOW);
  digitalWrite(INB1,LOW);
  analogWrite(PWM1,0);

  digitalWrite(INA2,LOW);
  digitalWrite(INB2,LOW);
  analogWrite(PWM2,0);
  
  digitalWrite(INA3,LOW);
  digitalWrite(INB3,LOW);
  analogWrite(PWM3,0);
  
  digitalWrite(INA4,LOW);
  digitalWrite(INB4,LOW); 
  analogWrite(PWM4,0);

  while(!Serial) continue;
  
  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0

} // End SetUp


//======================= loop() ==========================


void loop() {
  if(Serial.available()) {
    DeserializationError error = deserializeJson(doc, Serial);
    Serial.println("serial available");
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    if(!error) {
      int dir = doc["dir"];
      Serial.println(dir);
      switch (dir) {
        // go forward
        case 0:
          digitalWrite(ledPin1, HIGH);
          moveForwardBackward(movement);
          break;
        // go right
        case 1:
          digitalWrite(ledPin2, HIGH);
       	  moveSideways(movement); 
          break;
        // go backward
        case 2:
          digitalWrite(ledPin3, HIGH);
          moveForwardBackward(-movement);
          break;
        // go left
        case 3:
          digitalWrite(ledPin4, HIGH);
           moveSideways(-movement); 
          break;
        default: 
          moveSideways(0);
          moveForwardBackward(0);
          rotate(0);
      }
    }
  }
} // End loop


//=============== Function Declaration ===============

// ======= Limit value to threshold =======
  int limitThreshold(int value, int threshold) 
  {
    return max(min(value, threshold), -1*threshold);
  }

// ======== moveForwardBack ===========
  void moveForwardBackward(int value) {
    int otm1 = OutputToMotor1(-value);
    int otm2 = OutputToMotor2(-value);
    int otm3 = OutputToMotor3(value);
    int otm4 = OutputToMotor4(value);
    analogWrite(PWM1,otm1);
    analogWrite(PWM2,otm2);
    analogWrite(PWM3,otm3);
    analogWrite(PWM4,otm4);
  }

// ========= moveSideways =========
  void moveSideways(int value) {
    analogWrite(PWM1,OutputToMotor1(value));
    analogWrite(PWM2,OutputToMotor2(-value));
    analogWrite(PWM3,OutputToMotor3(-value));
    analogWrite(PWM4,OutputToMotor4(value));
  }

// ======== rotate =====
  void rotate(int value) {
    analogWrite(PWM1,OutputToMotor1(-value));
    analogWrite(PWM2,OutputToMotor2(-value));
    analogWrite(PWM3,OutputToMotor3(-value));
    analogWrite(PWM4,OutputToMotor4(-value));
  }

//===== int OutputToMotor(int value) ======
  //===== Assign Motor's Direction and Scale Down Input Signal =====
  // value must be positive and scaled down to fit 8-Bit PWM Range. 

  // Motor 1
  int OutputToMotor1(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      // CW
      digitalWrite(INA1,LOW);
      digitalWrite(INB1,HIGH);
      temp = map(value,0,MOVEMENT_THRESHOLD,0,255);
    }else{
      // CCW
      digitalWrite(INA1,HIGH);
      digitalWrite(INB1,LOW);
      temp = map(-value,0,MOVEMENT_THRESHOLD,0,255);
    }
    return temp;
  }
  
  // Motor 2
  int OutputToMotor2(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA2,LOW);
      digitalWrite(INB2,HIGH);
      temp = map(value,0,MOVEMENT_THRESHOLD,0,255);
    }else{
      digitalWrite(INA2,HIGH);
      digitalWrite(INB2,LOW);
      temp = map(-value,0,MOVEMENT_THRESHOLD,0,255);
    }
    return temp;
  }
  
  // Motor 3
  int OutputToMotor3(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA3,LOW);
      digitalWrite(INB3,HIGH);
      temp = map(value,0,MOVEMENT_THRESHOLD,0,255);
    }else{
      digitalWrite(INA3,HIGH);
      digitalWrite(INB3,LOW);
      temp = map(-value,0,MOVEMENT_THRESHOLD,0,255);
    }
    return temp;
  }
  
  // Motor 4
  int OutputToMotor4(int value)
  {
    int temp = 0;
    if(value >= 0)
    {
      digitalWrite(INA4,LOW);
      digitalWrite(INB4,HIGH);
      temp = map(value,0,MOVEMENT_THRESHOLD,0,255);
    }else{
      digitalWrite(INA4,HIGH);
      digitalWrite(INB4,LOW);
      temp = map(-value,0,MOVEMENT_THRESHOLD,0,255);
    }
    return temp;
  }
