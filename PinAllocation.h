/***********************************
* Filename: PinAllocation.h
* Purpose: Include pin definitions for our MECHENG 706 robot
* Date created: 3/May/2021
***********************************/

// Phototransistor pins
#define PHOTOTRANSISTOR1 A12 // LL
#define PHOTOTRANSISTOR2 A13 // LC
#define PHOTOTRANSISTOR3 A14 // RC
#define PHOTOTRANSISTOR4 A15 // RR

// Servo motor definitions ? 
#define SERVO_PIN 45
#define SERVO_MAX 160
#define SERVO_MIN 95
#define SERVO_MIDDLE 135

// Infrared rangefinder pins
#define IR_RR A5
#define IR_LR A4
#define IR_RF A7
#define IR_LF A6

// Motor pins
#define FL_MOTOR_PIN 46
#define FR_MOTOR_PIN 51
#define BL_MOTOR_PIN 47
#define BR_MOTOR_PIN 50

// Ultrasonic pins
#define ULTRASONIC_OUT 48
#define ULTRASONIC_IN 49

// Definitions used for WallDetected() function
// #define SIDE_WALL true
// #define FRONT_WALL false

// Fan pin
#define FAN_PIN 22

//Gyorscope Pin
#define GYRO_PIN A3
