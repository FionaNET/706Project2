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

#define FILTERLENGTH_P 5 // phtotransistor filter length
#define FILTERLENGTH_IR 5 // ir sensor filter length


// Servo motor definitions ? 
#define SERVO_PIN 48
#define SERVO_MAX 2000
#define SERVO_MIN 1000
#define SERVO_MIDDLE 1500

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
#define ObstacleSizeMax 260         //When doing the wall inverse cases
#define momentumTime 1000            //Strafing time for wall inverse cases

//isObject function thresholds
#define FrontObject 100
#define BackObject 100

#define IRNearThresh1 80
#define IRNearThresh2 180
#define USNearThresh1 100
#define USNearThresh2 200

//Obstacle avoid wall turning threshold
#define LFwallTurn 130

// Thresholds for detection the light 
#define TARGET_BRIGHTNESS 600
#define TARGET_BRIGHTNESS_OUT 650
#define TARGET_DISTANCE 300
#define SERVO_TARGET_BRIGHTNESS 100
#define DETECT_BRIGHTNES 30 // the sum of brightness for 4 phototransistors

//Threshold for lightdetect.h
#define LIGHT_THRESH_CLOSE 50
#define LIGHT_THRESH_FAR 15

// Threshold for rotate_while_scan
#define SEARCH_FINE_TUNE 500 // time of turning when light is detected
#define OUTPT_DIFF 100 // the difference of brightness between outside phototransistors



