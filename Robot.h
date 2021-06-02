#include "Gyroscope.h"
#include "IR_Sensor.h"
#include "UltrasonicSensor.h"
#include "RobotBase.h"
#include "LightDetect.h"
#include <Arduino.h>
#include "Firefighting.h"
#include <Servo.h>

class Robot{
    public:
        Robot();
        int check();
        int rotate_while_scan(bool dir);
        bool obstacle_Avoid();
        void CL_Turn(int angle);
        float NEAR(float dist, bool isIR);
        float FAR(float dist, bool isIR);
        float FuzzyRules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF);
        float Left_Rules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF);
        float Right_Rules(float LeftN, float LeftF, float RightF, float CenterN, float CenterF);
        float Forward_Rules(float LeftN, float RightN, float CenterN);
        void servoRotate();
        //int Defuzz(float direction);
        float min3(float a, float b, float c);
        bool go_target();
        LightDetect* lightInfo = new  LightDetect(); 
        RobotBase  wheels;
       // Firefighting fighter;
        IR_Sensor LF_IR, RF_IR, RR_IR, LR_IR;
        Gyroscope gyro;
        UltrasonicSensor sonar;
        bool invDirection;
        bool avoidanceOn;
        //int stopPos;
        int fanPin;
        //servo functions
        void servoRight();
        void servoLeft();
        void servoReset();
        void FanServoDisable();
        void FanServoAttach();
    private:
        Servo fanServo;

        float thr_sonar;
        bool backPass;
        bool passWait;  
        bool Strafed;
        float direction;
        unsigned long startTime;
        unsigned long stopTime;
        float memory;

        float speed_step; // this will get mupliplied by the fuzzy value

        float* fuzzify(float e);
        
        //float obstacleThresh
};