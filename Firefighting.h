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
    bool ExtinguishFire(void); //Main function to extinguish fire
    
    //deconstructor
    ~Firefighting();

    private: 
    //private variables 
    int fireThreshhold;
    LightDetect* LightDetector;

    //private functions
    void FanOn(void);
    void FanOff(void);
};

#endif