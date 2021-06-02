//Control all four wheels of the robot

#ifndef RobotBase_h
#define RobotBase_h

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "PinAllocation.h"
#define LEFT false
#define RIGHT true

class RobotBase {
  public:
    // Constructor
    RobotBase();

    // Methods that control robot movement
    //void Init(int LFpin, int RFpin, int LRpin, int RRpin);
    void Stop();
    void Disable();
    void norm(int speed);
    void Move(int angle, float speed);
    void Turn(bool direction, float speed);
    void Straight(float speed);
    void Strafe(bool direction, unsigned long time);
    void Print();
    void setMotors();
    void Attach();
    Servo fanServo;
    void FanServoDisable();
    void FanServoAttach();
    //void Norm();

  private:
    // Private members
    Servo LF, RF, LR, RR;
    int LFpin, RFpin, LRpin, RRpin, fanPin;
    int LFcurrent, RFcurrent, LRcurrent, RRcurrent;
     
};

#endif