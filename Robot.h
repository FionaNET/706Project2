#include <Gyroscope.cpp>
#include <IR_Sensor.cpp>
#include <Ultrasonic.cpp>
#include <RobotBase.cpp>
#include <LightDetect.cpp>
#include <Arduino.h>

class Robot{
    public:
        void Robot();
        void rotate_while_scan();
        void obstical_avoid();
        void go_target();
        void rotate_angle(float ref_angle); // turn to reference angle

        float alpha; // tuning parameter for rotation to a given angle
        float speed; // rotataing speed
    
    private:
        RobotBase wheels;
        LightDetect* lightInfo;
        IR_Sensor LF_IR, RF_IR, RR_IR, LR_IR;
        Gyroscope gyro;
        Ultrasonic sonar;
        bool backPass;
        //float obsticalThresh


        float ref_angle; // reference angle for turn right or left when correcting the path, get mupliplied by the fuzzy value
};