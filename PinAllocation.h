/***********************************
* Filename: PinAllocation.h
* Purpose: Include pin definitions for our MECHENG 706 robot
* Date created: 3/May/2021
***********************************/

// Phototransistor pins
#define PHOTOTRANSISTOR1 A8 // LL
#define PHOTOTRANSISTOR2 A9 // LC
#define PHOTOTRANSISTOR3 A10 // RC
#define PHOTOTRANSISTOR4 A11 // RR

#define FILTERLENGTH_P 5

// Servo motor definitions ? 
#define SERVO_PIN 45
#define SERVO_MAX 160
#define SERVO_MIN 95
#define SERVO_MIDDLE 135

// Infrared rangefinder pins
#define IR_RR A5   //40 to 300mm
#define IR_LR A4   //40 to 300mm
#define IR_RF A7   //100 to 800mm
#define IR_LF A6   //100 to 800mm

// Motor pins
#define FL_MOTOR_PIN 46 // Front left
#define FR_MOTOR_PIN 51 // Front right
#define BL_MOTOR_PIN 47 // back left
#define BR_MOTOR_PIN 50 // back right

// Ultrasonic pins
#define TRIG_PIN 49 //out
#define ECHO_PIN 48 // in

// Definitions used for WallDetected() function
// #define SIDE_WALL true
// #define FRONT_WALL false

// Fan pin
#define FAN_PIN 5

//Gyorscope Pin
#define GYRO_PIN A3

//Threshholds for obstacle avoidance
#define ObstacleSizeMax 250         //When doing the wall inverse cases
#define momentumTime 900            //Strafing time for wall inverse cases

//isObject function thresholds
#define FrontObject 120
#define BackObject 140

#define IRNearThresh1 80
#define IRNearThresh2 180
#define USNearThresh1 100
#define USNearThresh2 200

//Obstacle avoid wall turning threshold
#define LFwallTurn 130



