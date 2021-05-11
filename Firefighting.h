#include <Arduino.h>
#include <Servo.h>
#include <PinAllocation.h>
#include <LightDetect.h>

class Firefighting {
    public:
    //constructor
    Firefighting(int ThreshholdVal);

    //public variables
    
    //public functions
    bool ExtinguishFire(void); //Main function to extinguish fire
    
    //deconstructor
    ~Firefighting();

    private: 
    //private variables 
    int fireThreshhold;
    bool Fire_extinguish;
    LightDetect* LightDetector;

    //private functions
    void FanOn(void);
    void FanOff(void);
};