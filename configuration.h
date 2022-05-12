#include <Arduino.h>
//PIN Declaration

//Receiver
const int CH1 = 36;
const int CH2 = 38;
const int CH3 = 40;
const int CH4 = 42;
const int CH5 = 48;
const int CH6 = 49;

//Stepper Motor
const int MOTOR_INTERFACE_TYPE = 1;

const int ARM_STEPPER_MAX_SPEED = 1000;
const int ARM_STEPPER_SPEED = 500;
const int ARM_STEPPER_ACCELERATION = 30;
const int ARM_DIR_PIN = 34;
const int ARM_STEP_PIN = 32;

const int ARM_DC_IN_A = 21;
const int ARM_DC_IN_B = 20;


// Threshold
const int MOVEMENT_THRESHOLD = 500;
const int TOGGLE_THRESHOLD = 500;

// Receiver Threshold
const int CH1_THRESHOLD = MOVEMENT_THRESHOLD;
const int CH2_THRESHOLD = MOVEMENT_THRESHOLD;
const int CH3_THRESHOLD = MOVEMENT_THRESHOLD;
const int CH4_THRESHOLD = MOVEMENT_THRESHOLD;
const int CH5_THRESHOLD = TOGGLE_THRESHOLD;
const int CH6_THRESHOLD = TOGGLE_THRESHOLD;

// Servo min max angle
const int SERVO_MIN = 10;
const int SERVO_MAX = 60;

//Motor Driver 1
const int PWM1 = 13;
const int INA1 = 43;
const int INB1 = 47;
const int CS1 = A8;

//Motor Driver 2
const int PWM2 = 12;
const int INA2 = 39;
const int INB2 = 41;
const int CS2 = A9;

//Motor Driver 3
const int PWM3 = 10;
const int INA3 = 35;
const int INB3 = 37;
const int CS3 = A10;

//Motor Driver 4
const int PWM4 = 9;
const int INA4 = 31;
const int INB4 = 33;
const int CS4 = A11;

//LED
const int ledPin1 = 29;
const int ledPin2 = 27;
const int ledPin3 = 25;
const int ledPin4 = 23;

//Test Switch
const int buttonPin = 22;

//Servo Motor PWM
const int servo1 = 44;
const int servo2 = 46;
const int servo3 = 45;
const int servo4 = 11;
