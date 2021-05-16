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
    RobotBase(int LFpin, int RFpin, int LRpin, int RFpin);

    // Methods that control robot movement
    //void Init(int LFpin, int RFpin, int LRpin, int RRpin);
    void Stop();
    void Disable();
    void norm(int speed);
    void Move(int angle, float speed);
    void Turn(bool direction, float speed);
    void Straight();
    void Strafe(bool direction, unsigned long time);
    void Print();

  private:
    // Private members
    Servo LF, RF, LR, RR;
    int LFcurrent, RFcurrent, LRcurrent, RRcurrent;
    void setMotors(); 
};

#endif