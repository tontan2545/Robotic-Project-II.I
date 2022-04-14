/* 
 * Set Board: Tools --> Board --> Arduino Mega2560
 * Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>
#include <Stepper.h>
#include <Ewma.h>
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
// Input Signal from Receiver 
  int input1 = 0;
  int input2 = 0;
  int input3 = 0;
  int input4 = 0;
  int input5 = 0;
  int input6 = 0;
// Output Signal to Motor Driver  
  int out1 = 0;
  int out2 = 0;
  int out3 = 0;
  int out4 = 0;
// Processed Input Signal
  int left_w = 0;
  int right_w = 0;
// Motor's Current
float currentValue1 = 0.0;
float currentValue2 = 0.0;
float currentValue3 = 0.0;
float currentValue4 = 0.0;
int currentLimit = 5;
// Exponential filtering
float filterRate = 0.1;
Ewma adcFilter1(filterRate);
Ewma adcFilter2(filterRate);
Ewma adcFilter3(filterRate);
Ewma adcFilter4(filterRate);
// Stepper Motor
Stepper gripperStepper(STEPS_PER_REVOLUTION , GRIPPER_PIN_1, GRIPPER_PIN_2, GRIPPER_PIN_3, GRIPPER_PIN_4);
Stepper armStepper(STEPS_PER_REVOLUTION , ARM_PIN_1, ARM_PIN_2, ARM_PIN_3, ARM_PIN_4);


//===================== setup() ========================


void setup() {
//===== Set Digital Pin Mode =====    
// Set pinmode to read command signal from Receiver.  
  pinMode(CH1,INPUT);   //channel 1
  pinMode(CH2,INPUT);   //channel 2
  pinMode(CH3,INPUT);   //channel 3
  pinMode(CH4,INPUT);   //channel 4
  pinMode(CH5,INPUT);   //channel 5
  pinMode(CH6,INPUT);   //channel 6  
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
  
// Set pinmode to write command signal to LED.
  pinMode(ledPin1,OUTPUT);
  pinMode(ledPin2,OUTPUT);
  pinMode(ledPin3,OUTPUT);
  pinMode(ledPin4,OUTPUT);
// Assign Servo variable to a servo pin
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo3.attach(servo3);
  myServo4.attach(servo4);


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

  // Initialize Servo Motor, Set servo to Mid-point.
  myServo1.write(10);
  myServo2.write(10);
  myServo3.write(10);
  myServo4.write(10);
     
  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0

  // Returns time(us)
  previousLoopTime = micros();

  //Set initial gripper rpm
  gripperStepper.setSpeed(GRIPPER_RPM);
  //Set initial arm rpm
  armStepper.setSpeed(ARM_RPM);

} // End SetUp


//======================= loop() ==========================


void loop() {
   
  loopTime = micros()-previousLoopTime;
  
  
  
  // if loop time is more than 10000 microseconds, do the next loop.
  // Limit Maximum feedback loop at 100Hz.
  if(loopTime >= 10000) 
  {
    // Set new loop time
    previousLoopTime = micros();

    
    // Read input signal from receiver. PulseIn signal from Receiver vary between 1000 - 2000.
    // Substract 1500 as the offset to set new signal range from (1000, 2000) to (-500, 500) 
    // Also set Deadband limit to the input Signal

    input1 = pulseIn(CH1,HIGH)-1500; //Channel 1
    input2 = pulseIn(CH2,HIGH)-1500; //Channel 2
    input3 = pulseIn(CH3,HIGH)-1500; //Channel 3
    input4 = pulseIn(CH4,HIGH)-1500; //Channel 4
    input5 = pulseIn(CH5,HIGH)-1500; //Channel 5
    input6 = pulseIn(CH6,HIGH)-1500; //Channel 6

    input1 = Deadband(input1,30); //Channel 1
    input2 = Deadband(input2,30); //Channel 2
    input3 = Deadband(input3,50); //Channel 3
    input4 = Deadband(input4,30); //Channel 4
    input5 = Deadband(input5,30); //Channel 5
    input6 = Deadband(input6,30); //Channel 6

    input1 = limitThreshold(input1, CH1_THRESHOLD);
    input2 = limitThreshold(input2, CH2_THRESHOLD);
    input3 = limitThreshold(input3, CH3_THRESHOLD);
    input4 = limitThreshold(input4, CH4_THRESHOLD);
    input5 = limitThreshold(input5, CH5_THRESHOLD);
    input6 = limitThreshold(input6, CH6_THRESHOLD);

    input1 = adcFilter1.filter(input1);
    input2 = adcFilter2.filter(input2);
    input3 = adcFilter3.filter(input3);
    input4 = adcFilter4.filter(input4);

    
    // Read Motor's Current From Motor Driver
    // The resolution of Arduino analogRead is 5/1024 Volts/Unit. (10-Bit, Signal vary from 0 to 1023 units)
    // The resolution of Current Sensor from POLOLU VNH5019 is 0.14 Volt/Amp.
    // Convert analogRead signal(Volt) to Current(Amp) by multiply (5/1024)/0.14 = 0.035 Amp/Unit.
    currentValue1 = analogRead(CS1)*0.035; // Motor Driver 1
    currentValue2 = analogRead(CS2)*0.035; // Motor Driver 2
    currentValue3 = analogRead(CS3)*0.035; // Motor Driver 3
    currentValue4 = analogRead(CS4)*0.035; // Motor Driver 4

    if(input5 > 0) {
      digitalWrite(ledPin3, LOW);
      digitalWrite(ledPin4, LOW);
      if(currentValue1 < currentLimit && abs(input1) > 40) {
        moveSideways(input1);
      }
      if(currentValue2 < currentLimit && abs(input2) > 40) {
        moveForwardBackward(input2);
      }
      if(currentValue4 < currentLimit && abs(input4) > 40) {
        rotate(input4);
      }
    }
    else {
      digitalWrite(ledPin3, HIGH);
      digitalWrite(ledPin4, HIGH);
      processGripperServo(input3, CH3_THRESHOLD, myServo3);
      processArmServo(input4, myServo4);
      Serial.println("Test");
      if(abs(input1) > MOVEMENT_THRESHOLD/2) {
        Serial.println(input1/abs(input1));
        gripperStepper.step(input1/abs(input1));
      }
      if(abs(input2) > MOVEMENT_THRESHOLD/2) {
        Serial.println("Test2222");
        armStepper.step(input2/abs(input2) * 15);
      }
    }

    
    

    // Print
    Serial.print("M1 ");
    Serial.print(input1);
    Serial.print(" M2 ");
    Serial.print(input2);
    Serial.print(" M3 ");
    Serial.print(input3);
    Serial.print(" M4 ");
    Serial.print(input4);
    Serial.print(" M5 ");
    Serial.print(input5);
    Serial.print(" M6 ");
    Serial.println(input6);
//    Serial.print("\t LoopTime ");
//    Serial.println(loopTime);
  } // End if
} // End loop


//=============== Function Declaration ===============

// ======= Limit value to threshold =======
  int limitThreshold(int value, int threshold) 
  {
    return max(min(value, threshold), -1*threshold);
  }

// ======= Process servo according to value =======
  void processGripperServo(int value, int threshold, Servo servo)
  {
    int mappedAngle = map(value, -1*threshold, threshold, SERVO_MAX, SERVO_MIN);
    servo.write(mappedAngle);
  }

// ======== Process arm servo according to value ======
  void processArmServo(int value, Servo servo) {
    int mappedAngle = map(value, -1*MOVEMENT_THRESHOLD, MOVEMENT_THRESHOLD, 0, 180);
    servo.write(mappedAngle);
  }

// ======== moveForwardBack ===========
  void moveForwardBackward(int value) {
    int otm1 = OutputToMotor1(value);
    int otm2 = OutputToMotor2(value);
    int otm3 = OutputToMotor3(-value);
    int otm4 = OutputToMotor4(value);
    analogWrite(PWM1,otm1);
    analogWrite(PWM2,otm2);
    analogWrite(PWM3,otm3);
    analogWrite(PWM4,otm4);
  }

// ========= moveSideways =========
  void moveSideways(int value) {
    analogWrite(PWM1,OutputToMotor1(-value));
    analogWrite(PWM2,OutputToMotor2(value));
    analogWrite(PWM3,OutputToMotor3(-value));
    analogWrite(PWM4,OutputToMotor4(-value));
  }

// ======== rotate =====
  void rotate(int value) {
    analogWrite(PWM1,OutputToMotor1(-value));
    analogWrite(PWM2,OutputToMotor2(-value));
    analogWrite(PWM3,OutputToMotor3(-value));
    analogWrite(PWM4,OutputToMotor4(value));
  }

//===== double Deadband(double value,double limit) =====
  //===== Set Dead Band =====
  // If the input signal from receiver is in the band limit, set input signal to 0.0.
  double Deadband(double value,double limit)
  {
    double temp = 0.0;
    if(value >= limit) temp = value-limit;
    else if(value <= -limit) temp = value+limit;
    else temp = 0.0;
    return temp;
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
