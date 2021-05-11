//Control the motion and all four wheels of the robot base

#include <RobotBase.h>

RobotBase::RobotBase(int LFpin, int RFpin, int LRpin, int RRpin) {
  LFcurrent = 1500;
  RFcurrent = 1500;
  LRcurrent = 1500;
  RRcurrent = 1500;

  LF.attach(LFpin);
  RF.attach(RFpin);
  LR.attach(LRpin);
  RR.attach(RRpin);
}

void RobotBase::Stop() {
  LFcurrent = 1500;
  RFcurrent = 1500;
  LRcurrent = 1500;
  RRcurrent = 1500;
  setMotors();
}

void RobotBase::Disable() {   //completelly disable the motors
  LF.detach();
  RF.detach();
  LR.detach();
  RR.detach();
}

// Angle from 0-360, speed from 0-100
// This ignores any current robot movement
void RobotBase::Move(int angle, float speed) {
  double FLpulse, FRpulse, BLpulse, BRpulse;

  if (speed > 100) {
	  speed = 100;
  }
}

void RobotBase::Straight(int speed){

  LFcurrent = 1500 + (speed - 20);
  LRcurrent = 1500 + speed;
  RFcurrent = 1500 - (speed-60);
  RRcurrent = 1500 - speed;
    
  setMotors();
}

void RobotBase::Norm(){
  int max_speed = 500;
  float norm = max(speed_changeLF, speed_changeRR);
  float norm2 = max(speed_changeRF, speed_changeLR);
  float max_speed = speed_val;
  norm = max(norm, norm2);
  speed_changeLF = (speed_changeLF / norm) * max_speed;
  speed_changeRR = (speed_changeRR / norm) * max_speed;
  speed_changeRF = (speed_changeRF / norm) * max_speed;
  speed_changeLR = (speed_changeLR / norm) * max_speed;
}

//Turns on the spot
void RobotBase::Turn(bool direction, int speed) {
	
  if (direction){   //true = turn right (clockwise direction)
    int speed_change = speed;
  }else{
    int speed_change = -speed;
  }

  LFcurrent = 1500 + speed;
  RFcurrent = 1500 + speed;
  LRcurrent = 1500 + speed;
  RRcurrent = 1500 + speed;
  setMotors();
}

void RobotBase::setMotors(){
  LF.writeMicroseconds(LFcurrent);
  RF.writeMicroseconds(RFcurrent);
  LR.writeMicroseconds(LRcurrent);
  RR.writeMicroseconds(RRcurrent);
}

void RobotBase::Strafe(bool direction, unsigned long time){
  int speed = 400;
  int start_time;
  if(time != 0){
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
  }else{
    start_time = millis();
    while(millis() - startTime < time){
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
    Stop();
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
