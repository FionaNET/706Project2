#ifndef Motions_h
#define Motions_h

#include "RobotBase.h"
#include "LightDetect.h"
#include <Arduino.h>
class Motions {
    public:
        Motions();
        void rotate_while_scan();
        void obstical_avoid();



    private: 
        LightDetect* lightInfo;
        float obsticalThresh;
        bool backTracker;   //Tracks position of back wheels in regards to obsitcal
        RobotBase* Robot;


};

#endif