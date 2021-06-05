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
    bool ExtinguishFire(); //Main function to extinguish fire
    
    //deconstructor
    ~Firefighting();

    LightDetect* lights = new  LightDetect(); 

    Servo fanServo;
    void FanServoDisable();
    void FanServoAttach();
    bool Servo_Rotate();
    void Servo_Reset();

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