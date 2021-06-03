#ifndef Firefighting_h
#define  Firefighting_h

#include <Arduino.h>
#include <Servo.h>
#include "PinAllocation.h"
#include "LightDetect.h"

class Firefighting {
    public:
    //constructor
    Firefighting(int ThreshholdVal);

    //public variables
    bool Fire_extinguish;
    
    //public functions
    bool ExtinguishFire(double angle); //Main function to extinguish fire
    
    //deconstructor
    ~Firefighting();

    LightDetect* lights = new  LightDetect(); 

    Servo fanServo;
    void FanServoDisable();
    void FanServoAttach();
    void servoLeft();
    void servoRight();
    bool servoRotate();
    void servoReset();
    void servoCall(int angle);

    private: 
    //private variables 
    int fireThreshhold;
    LightDetect* LightDetector;

    //private functions
    void FanOn(void);
    void FanOff(void);

    int fanPin;
};

#endif