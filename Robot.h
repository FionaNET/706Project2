#include "Gyroscope.h"
#include "IR_Sensor.h"
#include "UltrasonicSensor.h"
#include "RobotBase.h"
#include "LightDetect.h"
#include <Arduino.h>

class Robot{
    public:
        Robot();
        void rotate_while_scan();
        void obstical_avoid();
        void CL_Turn(int angle);
        float NEAR(float dist);
        float FAR(float dist);
        float FuzzyRules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF);
        float Left_Rules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF);
        float Right_Rules(float LeftN, float RightF, float CenterN, float CenterF);
        float Forward_Rules(float LeftN, float RightN, float CenterN);
        float min3(float a, float b, float c);
        void go_target();
    
    private:
        RobotBase  wheels;
        LightDetect* lightInfo = new  LightDetect(); 
        IR_Sensor LF_IR, RF_IR, RR_IR, LR_IR;
        Gyroscope gyro;
        UltrasonicSensor sonar;

        float thr_sonar;
        bool backPass;
        bool PassFlagOn;
        bool Strafed;
        float direction;
        unsigned long startTime;
        unsigned long stopTime;
        float memory;

        float speed_step; // this will get mupliplied by the fuzzy value

        float* fuzzify(float e);
        
        //float obsticalThresh
};