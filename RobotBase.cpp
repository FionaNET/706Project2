//Control the motion and all four wheels of the robot base

#include "RobotBase.h"

RobotBase::RobotBase() {
  this->LFcurrent = 1500;
  this->RFcurrent = 1500;
  this->LRcurrent = 1500;
  this->RRcurrent = 1500;

  this->LFpin = FL_MOTOR_PIN;
  this->RFpin =  FR_MOTOR_PIN;
  this->LRpin =  BL_MOTOR_PIN;
  this->RRpin =  BR_MOTOR_PIN;
  this->fanPin = SERVO_PIN;
}

void RobotBase::Attach() {
  LF.attach(this->LFpin);
  RF.attach(this->RFpin);
  LR.attach(this->LRpin);
  RR.attach(this->RRpin);
}

void RobotBase::Stop() {
  this->LFcurrent = 1500;
  this->RFcurrent = 1500;
  this->LRcurrent = 1500;
  this->RRcurrent = 1500;
  setMotors();
}

void RobotBase::FanServoDisable() { 
  fanServo.detach();
  pinMode(this->fanPin, INPUT);
}

void RobotBase::FanServoAttach() { 
  fanServo.attach(this->fanPin);
  //pinMode(this->fanPin, OUTPUT);
}

void RobotBase::Disable() {   //completelly disable the motors
  LF.detach();
  RF.detach();
  LR.detach();
  RR.detach();

  pinMode(this->LFpin, INPUT);
  pinMode(this->RFpin, INPUT);
  pinMode(this->RRpin, INPUT);
  pinMode(this->LRpin, INPUT);
}

// Angle from 0-360, speed from 0-100
// This ignores any current robot movement
void RobotBase::Move(int angle, float speed) {
  double FLpulse, FRpulse, BLpulse, BRpulse;

  if (speed > 100) {
	  speed = 100;
  }
}

void RobotBase::Straight(float speed){

  this->LFcurrent = 1500 + (speed - 20);
  this->LRcurrent = 1500 + speed;
  this->RFcurrent = 1500 - (speed-40);    //-60
  this->RRcurrent = 1500 - speed;
    
  this->setMotors();
}

//void RobotBase::Norm(){
// int max_speed = 500;
//  float norm = max(speed_changeLF, speed_changeRR);
//  float norm2 = max(speed_changeRF, speed_changeLR);
//  float max_speed = speed_val;
//  norm = max(norm, norm2);
//  speed_changeLF = (speed_changeLF / norm) * max_speed;
//  speed_changeRR = (speed_changeRR / norm) * max_speed;
//  speed_changeRF = (speed_changeRF / norm) * max_speed;
//  speed_changeLR = (speed_changeLR / norm) * max_speed;
//}

//Turns on the spot
void RobotBase::Turn(bool direction, float speed) {
	int speed_change;
  if (direction){   //true = turn right (clockwise direction)
    speed_change = speed;
  }else{
    speed_change = -speed;
  }

  this->LFcurrent = 1500 + speed_change;
  this->RFcurrent = 1500 + speed_change;
  this->LRcurrent = 1500 + speed_change;
  this->RRcurrent = 1500 + speed_change;

  setMotors();
}

void RobotBase::setMotors(){
  LF.writeMicroseconds(this->LFcurrent);
  RF.writeMicroseconds(this->RFcurrent);
  LR.writeMicroseconds(this->LRcurrent);
  RR.writeMicroseconds(this->RRcurrent);
}

void RobotBase::Strafe(bool direction, unsigned long time){
  int speed = 200;
  int start_time;
  if(time != 0){        //want to strafe with a time requirement 
    start_time = millis();
    while((millis() - start_time) < time){
      if(direction){  //strafe right
        LFcurrent = 1500 + speed;
        RFcurrent = 1500 + speed;
        LRcurrent = 1500 - speed;
        RRcurrent = 1500 - speed;
      }else{        //strafe left
        LFcurrent = 1500 - speed;
        RFcurrent = 1500 - speed;
        LRcurrent = 1500 + speed;
        RRcurrent = 1500 + speed;
      }
      setMotors();
    }
    
  }else{      //just strafing
    if(direction){  //strafe right
      LFcurrent = 1500 + speed;
      RFcurrent = 1500 + speed;
      LRcurrent = 1500 - speed;
      RRcurrent = 1500 - speed;
    }else{        //strafe left
      LFcurrent = 1500 - speed;
      RFcurrent = 1500 - speed;
      LRcurrent = 1500 + speed;
      RRcurrent = 1500 + speed;
    }
      setMotors();
    }
  
}

void RobotBase::Print() {
  Serial.print(LFcurrent);
  Serial.print(", ");
  Serial.print(RFcurrent);
  Serial.print(", ");
  Serial.print(LRcurrent);
  Serial.print(", ");
  Serial.println(RRcurrent);
}
