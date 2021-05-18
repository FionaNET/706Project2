//Control all four wheels of the robot

#ifndef RobotBase_h
#define RobotBase_h

#include <Arduino.h>
#include <Servo.h>
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
    //void Norm();

  private:
    // Private members
    Servo LF, RF, LR, RR;
    int LFpin, RFpin, LRpin, RRpin;
    int LFcurrent, RFcurrent, LRcurrent, RRcurrent;
    void setMotors(); 
};

#endif