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
    
    private:
        RobotBase wheels;
        LightDetect* lightInfo = LightDetect(); 
        IR_Sensor LF_IR, RF_IR, RR_IR, LR_IR;
        Gyroscope gyro;
        Ultrasonic sonar;
        bool backPass;
        //float obsticalThresh
};