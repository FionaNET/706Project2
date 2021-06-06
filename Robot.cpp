#include "Robot.h"
#include <Arduino.h>
#include <math.h>


#define obstacleThresh 150
#define directionThresh 15

Robot::Robot(){
  this->LF_IR = IR_Sensor(LONG,IR_LF);
  this->RF_IR = IR_Sensor(LONG,IR_RF);
  this->LR_IR = IR_Sensor(MID,IR_LR);
  this->RR_IR = IR_Sensor(MID,IR_RR);


  this->wheels = RobotBase();
 // this->fighter = Firefighting(15);

  this->gyro = Gyroscope();
  //gyro.GyroscopeCalibrate();
  this->sonar = UltrasonicSensor();
  this->passWait = false;
  this->Strafed = false;
  this->thr_sonar = 10; // need to adjust
  this->speed_step = 500;
  //this->stopPos = 0;

  this->avoidanceOn = false;
  this->invDirection = false;
  this->LightFlag = false;
  this->ScanFlag = false;
}

// Function that will rotate until the robot detects a light source
// return 1 = successfully detected light
// return 2 = stopped due to timeout
int Robot::Rotate_While_Scan(bool dir){
    float LLAve = this->lightInfo->PT_LL->getRawReading();
    float LCAve = this->lightInfo->PT_LC->getRawReading();
    float RCAve = this->lightInfo->PT_RC->getRawReading();
    float RRAve = this->lightInfo->PT_RR->getRawReading();
  int speed;
  if (dir){ // true, turn right, CW
    speed = 150;
  }else{
    // false, turn left, CCW
    speed = -150;
  }

  Serial.println("Rotating while scan...");
  //bool front = lightInfo->detect_front();

  float timeStart = millis();
  float timeout = 9000; // 5 sec for a full rotation
  bool front = lightInfo->detect_front();
  //while (!front && ((LLAve+LCAve+RCAve+RRAve)>DETECT_BRIGHTNES)) {
  while (!front) {
    //Serial.println("Rotate while scanning loop");
    this->wheels.Turn(true, speed); // turn right 360 deg and scan
    front = lightInfo->detect_front();
    if (millis()-timeStart > timeout) {
      return 2;
    }
  }

  LLAve = this->lightInfo->PT_LL->getRawReading();
  LCAve = this->lightInfo->PT_LC->getRawReading();
  RCAve = this->lightInfo->PT_RC->getRawReading();
  RRAve = this->lightInfo->PT_RR->getRawReading();
  if ((LLAve -RRAve)> OUTPT_DIFF){
    // turn left a little bit to face the light
    dir = false;
    speed = -100;
    this->wheels.Turn(dir, speed);
    delay(SEARCH_FINE_TUNE);

  }else if ((RRAve-LLAve)>OUTPT_DIFF){
    // turn right a little bit to face the light
    dir = true;
    speed = 100;
    this->wheels.Turn(dir, speed);
    delay(SEARCH_FINE_TUNE);
  }

  //Serial.println("In rotate_while_scan, found light, should go target");
  return 1;
}

bool Robot::Obstacle_Avoid(){

  float d1, d2, d3, d4, d5, LeftMax, ForwardMax, RightMax;
  float LLAve = this->lightInfo->PT_LL->getRawReading();
  float LCAve = this->lightInfo->PT_LC->getRawReading();
  float RCAve = this->lightInfo->PT_RC->getRawReading();
  float RRAve = this->lightInfo->PT_RR->getRawReading();

  float distLC = this->lightInfo->PT_LC->getDistance();
  float distRC = this->lightInfo->PT_RC->getDistance();
  bool close = true;
  //bool LightFlag = false; //Determines when obstacle avoid has been completed, (determines if close to light)

  //Closer to the light, higher the ADC value
  //We want to adjust the direction of the robot the robot is far from the light
  if ( ((distLC+distRC)/2 < TARGET_DISTANCE) || ((LCAve +RCAve)/2 >= TARGET_BRIGHTNESS) || (RRAve >= TARGET_BRIGHTNESS_OUT_R) || (LLAve >= TARGET_BRIGHTNESS_OUT_L)) {
    LightFlag = true;
    avoidanceOn = false;
  }else{
    LightFlag = false;
  }
  ScanFlag = false;

  float RF_read = RF_IR.getReading();
  float LF_read = LF_IR.getReading();
  float RR_read = RR_IR.getReading();
  float LR_read = LR_IR.getReading();
  float sonar_read = sonar.ReadUltraSonic();
  float front_avg = ((RF_read + LF_read + sonar_read)/3);

  if(!LightFlag){
    if(front_avg < 130) {
      this->CL_Turn(160);
      delay(100);
      ScanFlag = true;
      //return true;
      //retFlag = true;
    //Both left sensors and front sensor
    }else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)) {
      this->CL_Turn(50);
      delay(100);
      ScanFlag = true;
    //Both right sensors and front sensor
    }else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)){
      this->CL_Turn(-50);
      delay(100);
      ScanFlag = true;
    }else {
      d1 = LF_IR.getReading();
      d2 = RF_IR.getReading();
      d3 = sonar.ReadUltraSonic();

      LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));

      //New method of including the forward readings
      if((ForwardMax > LeftMax) || ForwardMax > RightMax){
        this->direction = 0;
      }else if(ForwardMax > LeftMax){
        this->direction = (RightMax - ForwardMax)*50;
      }else if(ForwardMax > RightMax){
        this->direction = (LeftMax - ForwardMax)* -50;
      }else {
        this->direction = (LeftMax - ForwardMax)*-50 + (RightMax - ForwardMax)*50;
      }

      //Movement commands
      if (direction > directionThresh)
      { //Strafe right
        avoidanceOn = true;
        if (!Strafed)
        {                       //Went straight last time
          startTime = millis(); //Count how long we have strafed for
          if (LF_IR.isObject())
          { //Left IR senses object
            if (sonar.isObject() && (RR_IR.getReading() < ObstacleSizeMax))
            { //Close to a wall and sonar and LF_IR on object
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime / 2); //Strafe a little to the other side
            }
            else if (!sonar.isObject() && (RR_IR.getReading() < (ObstacleSizeMax)))
            { //Close to a wall and LF_IR on object
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime); //Strafe a lot to the other side
            }
            else
            {
              invDirection = false;
              wheels.Strafe(RIGHT, 0); //No wall (no need to change strafe direction) keep strafing right
            }
          }
          else if (sonar.isObject()) //Close to a wall and sonar on object
          {
            if (RR_IR.getReading() < (ObstacleSizeMax))
            { //Strafe a little to the other side
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime / 2);
            }
            else
            { //No wall (no need to change strafe direction) keep strafing right
              invDirection = false;
              wheels.Strafe(RIGHT, 0);
            }
          }
          else
          {
            wheels.Strafe(RIGHT, 0);    //No wall (no need to change strafe direction) keep strafing right
          }
        }
        else
        {
          Strafed = true;              
          memory = this->direction;    
        }

        if (invDirection){
          memory = -(this->direction);
        }else{
          memory = this->direction;
        }

        Strafed = true;
      }
      else if (direction < -directionThresh)
      { //Strafe left
        this->avoidanceOn = true;
        if (!Strafed)
        {
          startTime = millis();
          if (RF_IR.isObject())
          {
            if (sonar.isObject() && (LR_IR.getReading() < ObstacleSizeMax))
            {
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime / 2);
            }
            else if (!sonar.isObject() && (LR_IR.getReading() < (ObstacleSizeMax / 2)))
            {
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime);
            }
            else
            {
              invDirection = false;
              wheels.Strafe(LEFT, 0);
            }
          }
          else if (sonar.ReadUltraSonic() < 220)
          {
            if (LR_IR.getReading() < ObstacleSizeMax)
            {
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime / 2);
            }
            else
            {
              invDirection = false;
              wheels.Strafe(LEFT, 0);
            }
          }
          else
          {
            wheels.Strafe(LEFT, 0);
          }
        }
        else
        {
          Strafed = true;
          memory = this->direction;
        }

        if (invDirection){
          memory = -(this->direction);
        }else{
          memory = this->direction;
        }

        Strafed = true;
        // Serial.println("obstacle avoid strafe left");
        // memory = this->direction;
      }
      else
      { //Going forward
        if (Strafed)
        { //On previous loop the car had strafed to avoid
          stopTime = millis();
          Strafed = false;
          passWait = true;
        }
        if (avoidanceOn)
        {                       //if it is in the process of avoiding an obstacle
          wheels.Straight(200); //go straight
          Serial.println("no obsticle go straight");
        }
        else
        {
          Serial.println("OBSTACLE AVOID: deattaching wheel"); //Detach wheel so robot completely stops (in main it will do go target)
          wheels.Disable();
        }
      }
      //memory = direction;       //Store initial strafe direction (so we know where to strafe back)
    Serial.println("Passwait = " + String(passWait));
      //Waiting for car to pass the obstacal and strafe back
      if(passWait){
        //turn flag off once the back IR sensors read the obstical (meaning we have passed it)
        Serial.println(String(memory));
        if(memory < -directionThresh){   //Strafed left at start
          Serial.println("Obstacle on left IR");
          Serial.println("RR ir reading = " + String(RR_IR.getReading()));
          passWait = !(RR_IR.getReading() < obstacleThresh);     //back ir detects object we have passed it
          if(!passWait){
            delay(StraightTime);               //wait for back wheel to pass obstical
            wheels.Strafe(RIGHT, (stopTime - startTime));      //Strafe to correct path
            this->avoidanceOn = false;
            Serial.println("strafe back left"); 
          }
        }else if(memory > directionThresh)
        {
          passWait = !(LR_IR.getReading() < obstacleThresh);
          Serial.println("Obstacle on right IR");
          if(!passWait){  //Once obstical has passed
          Serial.println("Passwait back right");
            delay(StraightTime);                                      //wait for back wheel to pass obstical
            wheels.Strafe(LEFT, (stopTime - startTime));    //Strafe back
            this->avoidanceOn = false;
            Serial.println("strafe back right");
          }
        }
      }
      //memory = direction;       //Store initial strafe direction (so we know where to strafe back)
    }
  }
  Serial.println("avoidance is" + String(avoidanceOn));
  return avoidanceOn; //true means it is avoiding, false means it is not avoiding
}


float Robot::NEAR(float dist, bool isIR){    //Fuzzificaiton - Get near membership value
  float thresh1,thresh2;
  if (isIR){                      //IR and ultrasonic sensors have different threshholds
    thresh1 = IRNearThresh1;
    thresh2 = IRNearThresh2;
  }else{
    thresh1 = USNearThresh1;
    thresh2 = USNearThresh2;
  }
  
  float g = -1/(thresh2 - thresh1);
  float c = 1 + (-g)*thresh1;

  if(dist <= thresh1){
    return 1.0;
  }else if(dist < thresh2 && dist > thresh1){
    return (g*dist + c);
  }else{
    return 0.0;
  }
}

float Robot::FAR(float dist, bool isIR){     //Fuzzification - Get far membership value
  float thresh1,thresh2;
  if (isIR){
    thresh1 = IRNearThresh1;
    thresh2 = IRNearThresh2;
  }else{
    thresh1 = USNearThresh1;
    thresh2 = USNearThresh2;
  }
  float g = 1/(thresh2 - thresh1);
  float c = (-g)*thresh1;

  if(dist <= thresh1){
    return 0.0;
  }else if(dist < thresh2 && dist > thresh1){
    return (g*dist + c);
  }else{
    return 1.0;
  }
}

float Robot::Left_Rules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF){
  float B = min3(LeftN, CenterF, RightN);
  float D = min3(LeftF, CenterN, RightN);
  float F = min3(LeftF, CenterF, RightN);

  float temp2 = max(B,D);
  return max(F, temp2);           //Return max of the same rules
  
}

float Robot::Right_Rules(float LeftN, float LeftF, float RightF, float CenterN, float CenterF){
  float A = min3(LeftN, CenterN, RightF);     
  float C = min3(LeftN, CenterF, RightF);
  float E = min3(LeftF, CenterN, RightF);
  
  float temp1 = max(A,C);
  return max(temp1,E);                //Return max of the same rules
}

float Robot::Forward_Rules(float LeftN, float RightN, float CenterN){ 
  float G = min3(LeftN, CenterN, RightN);
  return G;                       //Return max of the same rules
}

float Robot::min3(float a, float b, float c){
  //get minimum of 3 values
  if(a <= b && a <= c){
    return a;
  }else if(b <= c){
    return b;
  }else{
    return c;
  }
}




void Robot::CL_Turn(int ref_angle){
  float kp_angle = 4;
  float ki_angle = 0.8;
  float cumulation;
  bool CCW = false;
  bool CW = true;
  float angle_thres = 5;
  //speed value added to the motors 
  float rot_change, angle_error; 

  float turn_start = millis();
  float turn_time = 0;

  gyro.currentAngle = 0;      //reset the current angle in gyro

  angle_error = ref_angle - gyro.GyroRead();

  while ((abs(angle_error) > angle_thres) && (turn_time <= 2000)) {       //will exit while loop if the time in the while loop exceeds 2sec
  
    if(abs(angle_error) < 12.0){
      cumulation = cumulation + angle_error;
    }

    //noticed the robot had a problem where at the start of turning, it would think it'll need to turn CCW due to a negative angle error
    //since the robot is always turning CW, we absolute the rot_change for large angle_error 
    if((ref_angle > 0) && abs(angle_error) > 30){
      rot_change = abs(kp_angle * angle_error + ki_angle*cumulation);
    }else if((ref_angle < 0) && abs(angle_error) > 30){
      rot_change = -1 * abs(kp_angle * angle_error + ki_angle*cumulation);
    }else{
      rot_change = kp_angle * angle_error + ki_angle*cumulation;
    } 

    rot_change = constrain(rot_change, -500, 500);
    
    // Serial.print("The rot_change is: ");
    // Serial.println(rot_change);
    
    //set the motors
    if(rot_change < 0){
      wheels.Turn(CCW, abs(rot_change));
    }else{
      wheels.Turn(CW, rot_change);
    }

    //read the gyro sensor and get current angle
    angle_error = ref_angle - gyro.GyroRead();
    // Serial.print("The currentAngle is: ");
    // Serial.println(gyro.GyroRead());
    // Serial.print("The angle error is: ");
    // Serial.println(angle_error);
    turn_time = millis() - turn_start;
  }
}


//-------------------TRYING OLD GO TARGET---------------------------//
bool Robot::Go_Target(){
  bool dir; 
  float Kp = 0.15;
  float Ki = 0.001;
  float accumulation = 0;
  float distLC = 0;
  float distRC = 0;
  float error = 0;
  float RLerror = 0;
  int search;
  float speed, ultraSpeed;

  //search = this->rotate_while_scan(true); // initial searching

  float LLAve = this->lightInfo->PT_LL->getRawReading();
  float LCAve = this->lightInfo->PT_LC->getRawReading();
  float RCAve = this->lightInfo->PT_RC->getRawReading();
  float RRAve = this->lightInfo->PT_RR->getRawReading();
  

  // Serial.print("Average of the middle 2 phototransistors:    ");
  // Serial.println((RCAve+LCAve)/2);

  distLC = this->lightInfo->PT_LC->getDistance();
  distRC = this->lightInfo->PT_RC->getDistance();

  if ( (distLC+distRC)/2 > 210 ) { // distance from the centre phototransistor values
    // this->obstacle_Avoid();
    
    LLAve = this->lightInfo->PT_LL->getRawReading();
    LCAve = this->lightInfo->PT_LC->getRawReading();
    RCAve = this->lightInfo->PT_RC->getRawReading();
    RRAve = this->lightInfo->PT_RR->getRawReading();


    // Fixes trajectory when light is somewhere but we might be off
    if (((RCAve+RRAve+LCAve+LLAve)/4) < 20){ 
      // Rotate to the highest light value
      if (RRAve>LLAve){
        dir = true;  // turn right
      } else{
        dir = false; // turn left
      }
      search = this->Rotate_While_Scan(dir);
    }
      
    RLerror = RRAve - LLAve;

    // Fixes problem when the robot is placed centre between two lights and starts driving straight to the centre
    // RR and LL reads very high at similar values and LC and RC reads low
    if ((abs(RLerror) < 60) && (RCAve < 20) && (LCAve < 20)) {   
      // Rotate to the highest light value
      if (RRAve>LLAve){
        dir = true;  // turn right
      }else{
        dir = false; // turn left
      }
      this->wheels.Turn(dir, 200); //turn the robot
      delay(550);                  //give time for the robot to turn away
    }

    float start_time = millis();    
    error = RCAve+RRAve - LCAve-LLAve; 
    // Change direction if the error is too higher or the time spent in the loop is less than 2 sec
    while  ((abs(error) > 350) || ((millis()-startTime)<2000)) { 
      // Only accumulate error if it is small
      if(abs(error) < 365){   // Previously we had a value of 20, but it would never enter the while loop with 20
        accumulation = accumulation + error;
      }
      speed = Kp*error + Ki*accumulation;
      constrain(speed, -500 , 500);

      // Determine the direction we are supposed to turn
      if (error > 0){
        // Positive error means right side is brighter
        dir = true;
      } else {
        // Negative error means left side is brighter
        dir = false;
      }

      this->wheels.Turn(dir, speed);
      LLAve = this->lightInfo->PT_LL->getRawReading();
      LCAve = this->lightInfo->PT_LC->getRawReading();
      RCAve = this->lightInfo->PT_RC->getRawReading();
      RRAve = this->lightInfo->PT_RR->getRawReading();
      error = RCAve+RRAve - LCAve-LLAve;
      // Serial.print("Updated error:    "); 
      // Serial.println(error);
    }

    this->wheels.Straight(150);
    return false;

  // Reached the light    
  } else {
    return true;
  }

}

