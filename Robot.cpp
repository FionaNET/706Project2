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
int Robot::rotate_while_scan(bool dir){
    float LLAve = this->lightInfo->PT_LL->getRawReading();
    float LCAve = this->lightInfo->PT_LC->getRawReading();
    float RCAve = this->lightInfo->PT_RC->getRawReading();
    float RRAve = this->lightInfo->PT_RR->getRawReading();
  int speed;
  if (dir){ // true, turn right, CW
    speed = 100;
  }else{
    // false, turn left, CCW
    speed = -100;
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
    speed = -50;
    this->wheels.Turn(dir, speed);
    delay(SEARCH_FINE_TUNE);

  }else if ((RRAve-LLAve)>OUTPT_DIFF){
    // turn right a little bit to face the light
    dir = true;
    speed = 50;
    this->wheels.Turn(dir, speed);
    delay(SEARCH_FINE_TUNE);
  }

  //Serial.println("In rotate_while_scan, found light, should go target");
  return 1;
}

//non fuzzy logic
// void Robot::obstical_avoid(){
//     //Read all IR_Sensors and decide if any is too much
//     unsigned long startTime, strafeTime;

//     if(LF_IR.getReading() < obstacleThresh){
//         backPass = false;
//         //Strafe right until the path is clear in front of robot
//         while(LF_IR.getReading() < obstacleThresh){
//             startTime = millis();
//             wheels.Strafe(RIGHT, 0);
//         }
//         strafeTime = millis() - startTime;
//         //Keep going forward until the back has passed
//         while(!backPass){
//             wheels.Straight(400);
//             backPass = (LR_IR.getReading() < obstacleThresh);
//         }
//         //Go back left
//         wheels.Strafe(LEFT, strafeTime);

//     }else if (RF_IR.getReading() < obstacleThresh){
//         backPass = false;
//         //Strafe right until the path is clear in front of robot
//         while(RF_IR.getReading() < obstacleThresh){
//             startTime = millis();
//             wheels.Strafe(LEFT, 0);
//         }
//         strafeTime = millis() - startTime;
//         while(!backPass){
//             wheels.Straight(400);
//             backPass = (LR_IR.getReading() < obstacleThresh);
//         }
//         //Go back to correct path once object is passed
//         wheels.Strafe(RIGHT, strafeTime);
//     }

int Robot::check(){
  int flag;
  float d1, d2, d3, d4, d5, LeftMax, ForwardMax, RightMax;
  
  if(LF_IR.isObject() && RF_IR.isObject() && sonar.isObject()){
    //All three sensors are reading objects so it is a wall
    flag = 1;
  }else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 150) && (sonar.ReadUltraSonic() < 200)) {
    flag = 2;
    //Serial.println("There is a wall on left so we are turning");
  }else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 150) && (sonar.ReadUltraSonic() < 200)){
    flag = 3;
  }else{
    //1 or more objects detected so it is a cyclindar
  
    //if(!this->passWait){      //Only do this once when no obsticles have been previously detected
    //get all distance readings
    d1 = LF_IR.getReading();
    d2 = RF_IR.getReading();
    d3 = sonar.ReadUltraSonic();
    //Serial.println("D1 = " + String(d1) + "  D2 = " + String(d2) + "  D3 = " + String(d3));

    LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
    RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
    ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));
    //Serial.println("LeftMax = " + String(LeftMax) + "  RightMax = " + String(RightMax) + "  ForwardMax = " + String(ForwardMax));

    //take weighted average

    //this->direction = LeftMax*-30 + ForwardMax*15 + RightMax*30;    //get direction
    //New method of including the forward readings
    if((ForwardMax > LeftMax) || ForwardMax > RightMax){
      this->direction = 0;
    }else if(ForwardMax > LeftMax){
      this->direction = (RightMax - ForwardMax)*50;
    }else if(ForwardMax > RightMax){
      this->direction = (LeftMax - ForwardMax)*(-50);
    }else {
      this->direction = (LeftMax - ForwardMax)*(-50) + (RightMax - ForwardMax)*50;
    }
    
    //Contstrain direction so it doesn't hit into a side wall by incorporating the readings from back
    //sensors
    // if(((LR_IR.getReading() < 130) && (direction < 0)) || ((RR_IR.getReading() < 130) && (direction > 0))){
    //   this->direction = this->direction*-1;
    // }
    //Serial.println("Direction = " + String(direction));
  

    //Movement commands
    if(direction > directionThresh){     //Strafe right
      avoidanceOn = true;           
      if(!Strafed){               //Went straight last time or strafed in opposite direction
        startTime = millis();     //Count how long we have strafed for
        //Serial.println("detect object left = " + String(LF_IR.isObject()));
        if(LF_IR.isObject()){
          flag = 4;
          if(sonar.isObject() && (RR_IR.getReading() < ObstacleSizeMax)){
            flag = 5;
          }else if(!sonar.isObject() && (RR_IR.getReading() < (ObstacleSizeMax))){
            flag = 6;
          }else{
            flag = 7;
          }
        }else if (sonar.isObject()){
          if(RR_IR.getReading() < (ObstacleSizeMax)){
            flag = 8;
          }else{
            flag = 9;
          }
        }else{
          flag = 10;
        }
      }else{
        flag = 11;
        Strafed = false;
      }
      Strafed = true;
    }else if(direction < -directionThresh){    //Strafe left
      this->avoidanceOn = true;
      if(!Strafed){
        startTime = millis();
        //Serial.println("detect object right = " + String(RF_IR.isObject()));
        if(RF_IR.isObject()){
          flag = 12;
          if(sonar.isObject() && (LR_IR.getReading() < ObstacleSizeMax)){
            flag = 13;
          }else if(!sonar.isObject() && (LR_IR.getReading() < (ObstacleSizeMax/2))){
            flag = 14;
          }else{
            flag = 15;
          }
        }else if (sonar.ReadUltraSonic() < 200) {
          if(LR_IR.getReading() < ObstacleSizeMax){
            flag = 16;
          }else{
            flag = 17;
          }
        }else{
          flag = 18;
        }
      }else{      //no obstical on the right but strafing left
        Strafed = true;
        //Serial.println("obstacle avoid strafe left");
        memory = this->direction;
      }
      Strafed = true;
    }else{                        //Going forward
      flag = 0;
      Strafed = false;
    }
  }
  return flag;
}

// //obstacle avoidance with fuzzy logic
// bool Robot::obstacle_Avoid(){

//   float d1, d2, d3, d4, d5, LeftMax, ForwardMax, RightMax;
//   float LLAve = this->lightInfo->PT_LL->getRawReading();
//   float LCAve = this->lightInfo->PT_LC->getRawReading();
//   float RCAve = this->lightInfo->PT_RC->getRawReading();
//   float RRAve = this->lightInfo->PT_RR->getRawReading();

//   float distLC = this->lightInfo->PT_LC->getDistance();
//   float distRC = this->lightInfo->PT_RC->getDistance();
//   bool close = true;
//   bool retFlag = false; //Determines when obstacle avoid has been completed

//   //Closer to the light, higher the ADC value
//   //We want to adjust the direction of the robot the robot is far from the light
//   if ( ((distLC+distRC)/2 < 400) || ((LCAve +RCAve)/2 >= TARGET_BRIGHTNESS) || (RRAve >= TARGET_BRIGHTNESS_OUT) || (LLAve >= TARGET_BRIGHTNESS_OUT)) {
//     retFlag = true;
//   }

//   // Serial.println("LF_IR dist: " + String(LF_IR.getReading()) + "RF_IR dist: " + String(RF_IR.getReading()) + "Sonar dist: " + String(sonar.ReadUltraSonic()));
//   // Serial.println("LF OBject: " + String(LF_IR.isObject()) + "  RF object: " + String(RF_IR.isObject()) + "  center object: " + String(sonar.isObject()));
  
//   float RF_read = RF_IR.getReading();
//   float LF_read = LF_IR.getReading();
//   float RR_read = RR_IR.getReading();
//   float LR_read = LR_IR.getReading();
//   float sonar_read = sonar.ReadUltraSonic();
//   float front_avg = ((RF_read + LF_read + sonar_read)/3);

//   while (!retFlag) {
//     //All three sensors are reading objects so it is a wall

//     if(front_avg < 120) {
//       this->CL_Turn(160);
//       delay(100);
//       return true;
//       retFlag = true;


//     // if(LF_IR.isObject() && RF_IR.isObject()){
//     //   Serial.println("There is a wall");
//     //   this->CL_Turn(180); 
//     //   delay(100);
//     //   retFlag = true;
//     //Both left sensors and front sensor
//     }else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)) {
//     //}else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 100) && (sonar.ReadUltraSonic() < 200)) {
//       //Serial.println("There is a left corner");
//       this->CL_Turn(50);
//       delay(100);
//       return true;
//       retFlag = true;
//     //Both right sensors and front sensor
//     }else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)){
//     //}else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 100) && (sonar.ReadUltraSonic() < 200)){
//       //Serial.println("There is a right corner");
//       this->CL_Turn(-50);
//       delay(100);
//       return true;
//       retFlag = true;
//     //No or multiple sensosrs (strafe or straight motion only)
//     }else{
//       //get all distance readings
//       d1 = LF_IR.getReading();
//       d2 = RF_IR.getReading();
//       d3 = sonar.ReadUltraSonic();
//       //Serial.println("D1 = " + String(d1) + "  D2 = " + String(d2) + "  D3 = " + String(d3));

//       LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
//       RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
//       ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));
//       //Serial.println("LeftMax = " + String(LeftMax) + "  RightMax = " + String(RightMax) + "  ForwardMax = " + String(ForwardMax));

//       //take weighted average

//       //this->direction = LeftMax*-30 + ForwardMax*15 + RightMax*30;    //get direction
//       //New method of including the forward readings
//       if((ForwardMax > LeftMax) || ForwardMax > RightMax){
//         this->direction = 0;
//       }else if(ForwardMax > LeftMax){
//         this->direction = (RightMax - ForwardMax)*50;
//       }else if(ForwardMax > RightMax){
//         this->direction = (LeftMax - ForwardMax)*(-50);
//       }else {
//         this->direction = (LeftMax - ForwardMax)*(-50) + (RightMax - ForwardMax)*50;
//       }

//       //Movement commands
//       if(direction > directionThresh){     //Strafe right        
//       this->avoidanceOn = true;
//           //Went straight last time or strafed in opposite direction
//           Serial.println("detect object left = " + String(LF_IR.isObject()));
//           if(LF_IR.isObject()){
//             Serial.println("sonar object = " + String(sonar.isObject()));
//             Serial.println("RR IR = " + String(RR_IR.getReading()));
//             if (sonar.isObject() && (RR_IR.getReading() < ObstacleSizeMax)){
//               wheels.Strafe(LEFT, momentumTime);
//             }else if(!sonar.isObject() && (RR_IR.getReading() < (ObstacleSizeMax))){
//               wheels.Strafe(LEFT, momentumTime/2);
//             }else{
//               wheels.Strafe(RIGHT, 0);
//             }
//           }else if (sonar.isObject()){
//             if(RR_IR.getReading() < (ObstacleSizeMax)){
//               wheels.Strafe(LEFT, momentumTime/2);
//             }else{
//               wheels.Strafe(RIGHT, 0);
//             }
//           }else{
//             wheels.Strafe(RIGHT, 0);
//           }

//       }else if(direction < -directionThresh){    //Strafe left
//         this->avoidanceOn = true;
//           Serial.println("detect object right = " + String(RF_IR.isObject()));
//           if(RF_IR.isObject()){
//             Serial.println("sonar object = " + String(sonar.isObject()));
//             Serial.println("LR IR = " + String(LR_IR.getReading()));
//             if(sonar.isObject() && (LR_IR.getReading() < ObstacleSizeMax)){
//               wheels.Strafe(RIGHT, momentumTime);
//             }else if(!sonar.isObject() && (LR_IR.getReading() < (ObstacleSizeMax/2))){
//               wheels.Strafe(RIGHT, momentumTime/2);
//               Serial.println("Inversing direction - Only RF on object");
//             }else{
//               wheels.Strafe(LEFT, 0);
//               Serial.println("Keep direction - Only RF on object");
//             }
//           }else if (sonar.ReadUltraSonic() < 80) {
//             if(LR_IR.getReading() < ObstacleSizeMax){
//               wheels.Strafe(RIGHT, momentumTime/2);
//               Serial.println("Inversing direction - Both sonar and RF on object");
//             }else{
//               wheels.Strafe(LEFT, 0);
//               Serial.println("Keep direction - Both sonar and RF on object");
//             }
//           }else{
//             wheels.Strafe(LEFT, 0);
//             invDirection = false;
//           }

//       //no obstacle, go straight  
//       }else{              
//         //bool goStraightFlag = false;  
//         wheels.Straight(200);

//         // while ((!goStraightFlag)  && (avoidanceOn)){
//         while (this->avoidanceOn){
//           if(LR_IR.isObject() || RR_IR.isObject() || LF_IR.isObject() ||  RF_IR.isObject() || sonar.isObject()){
//             //goStraightFlag = true; //finished going straight
//             this->avoidanceOn = false;
//             Serial.println("Stopped going straight in obstacle avoidance");
//             return true;
//           }
//         }
//         retFlag = true; //going straight finished, get out of while loop
//       }

//     }

//   }
//   return false;
// }

bool Robot::obstacle_Avoid(){

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
  if ( ((distLC+distRC)/2 < 400) || ((LCAve +RCAve)/2 >= TARGET_BRIGHTNESS) || (RRAve >= TARGET_BRIGHTNESS_OUT_R) || (LLAve >= TARGET_BRIGHTNESS_OUT_L)) {
    LightFlag = true;
  }else{
    LightFlag = false;
  }
  ScanFlag = false;

  // Serial.println("LF_IR dist: " + String(LF_IR.getReading()) + "RF_IR dist: " + String(RF_IR.getReading()) + "Sonar dist: " + String(sonar.ReadUltraSonic()));
  // Serial.println("LF OBject: " + String(LF_IR.isObject()) + "  RF object: " + String(RF_IR.isObject()) + "  center object: " + String(sonar.isObject()));
  
  float RF_read = RF_IR.getReading();
  float LF_read = LF_IR.getReading();
  float RR_read = RR_IR.getReading();
  float LR_read = LR_IR.getReading();
  float sonar_read = sonar.ReadUltraSonic();
  float front_avg = ((RF_read + LF_read + sonar_read)/3);


  Serial.println("LF_IR dist: " + String(LF_IR.getReading()) + "RF_IR dist: " + String(RF_IR.getReading()) + "Sonar dist: " + String(sonar.ReadUltraSonic()));
  Serial.println("  LF OBject: " + String(LF_IR.isObject()) + "  RF object: " + String(RF_IR.isObject()) + "  center object: " + String(sonar.isObject()));
  
  while(!LightFlag){
    if(front_avg < 120) {
      this->CL_Turn(160);
      delay(100);
      ScanFlag = true;
      //return true;
      //retFlag = true;


    // if(LF_IR.isObject() && RF_IR.isObject()){
    //   Serial.println("There is a wall");
    //   this->CL_Turn(180); 
    //   delay(100);
    //   retFlag = true;
    //Both left sensors and front sensor
    }else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)) {
    //}else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 100) && (sonar.ReadUltraSonic() < 200)) {
      //Serial.println("There is a left corner");
      this->CL_Turn(50);
      delay(100);
      ScanFlag = true;
      //return true;
      //retFlag = true;
    //Both right sensors and front sensor
    }else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 180) && (sonar.ReadUltraSonic() < 220)){
    //}else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 100) && (sonar.ReadUltraSonic() < 200)){
      //Serial.println("There is a right corner");
      this->CL_Turn(-50);
      delay(100);
      ScanFlag = true;
      //return true;
      //retFlag = true;

    // if(LF_IR.isObject() && RF_IR.isObject() && sonar.isObject()){
    //   //All three sensors are reading objects so it is a wall
    //   this->CL_Turn(90);
    //   delay(100);
    //   this->avoidanceOn = false;
    //   //Serial.println("There is a wall in front so we are turning");

    // }else if((LF_IR.getReading() < 100) && (LR_IR.getReading() < 150) && (sonar.ReadUltraSonic() < 200)) {
    //   this->CL_Turn(45);
    //   delay(100);
    //   this->avoidanceOn = false;
    //   //Serial.println("There is a wall on left so we are turning");
    // }else if((RF_IR.getReading() < 100) && (RR_IR.getReading() < 150) && (sonar.ReadUltraSonic() < 200)){
    //   this->CL_Turn(-45);
    //   delay(100);
    //   this->avoidanceOn = false;
    // // Serial.println("There is a wall on right so we are turning");
    // }
    }else {
      //1 or more objects detected so it is a cyclindar
      //get all distance readings

      //if(!this->passWait){      //Only do this once when no obsticles have been previously detected
        d1 = LF_IR.getReading();
        d2 = RF_IR.getReading();

        d3 = sonar.ReadUltraSonic();
        //Serial.println("D1 = " + String(d1) + "  D2 = " + String(d2) + "  D3 = " + String(d3));

        LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
        RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
        ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));
        //Serial.println("LeftMax = " + String(LeftMax) + "  RightMax = " + String(RightMax) + "  ForwardMax = " + String(ForwardMax));

        //take weighted average

        //this->direction = LeftMax*-30 + ForwardMax*15 + RightMax*30;    //get direction
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
        
      //}
        //Contstrain direction so it doesn't hit into a side wall by incorporating the readings from back
        //sensors
        // if(((LR_IR.getReading() < 130) && (direction < 0)) || ((RR_IR.getReading() < 130) && (direction > 0))){
        //   this->direction = this->direction*-1;
        // }
        Serial.println("Direction = " + String(direction));
      

      //Movement commands
      if(direction > directionThresh){     //Strafe right
        avoidanceOn = true;           
        if(!Strafed){              //Went straight last time or strafed in opposite direction
          startTime = millis();     //Count how long we have strafed for
          Serial.println("detect object left = " + String(LF_IR.isObject()));
          if(LF_IR.isObject()){    //Left on sensor strafe a lot the otherway 
            Serial.println("sonar object = " + String(sonar.isObject()));
            Serial.println("RR IR = " + String(RR_IR.getReading()));
            if(sonar.isObject() && (RR_IR.getReading() < ObstacleSizeMax)){
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime/2);
            }else if(!sonar.isObject() && (RR_IR.getReading() < (ObstacleSizeMax))){
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime);
            }else{
              invDirection = false;
              wheels.Strafe(RIGHT, 0);
            }
        }else if (sonar.isObject())
        {
            if(RR_IR.getReading() < (ObstacleSizeMax)){
              invDirection = true;
              wheels.Strafe(LEFT, momentumTime/2);
            }else{
              invDirection = false;
              wheels.Strafe(RIGHT, 0);
            }
        }else{
            wheels.Strafe(RIGHT, 0);
        }
        }else{
          Strafed = true;
          Serial.println("obstacle avoid strafe right");
          memory = this->direction;
        }

        if(invDirection){
          memory = -(this->direction);
        }else{
          memory = this->direction;
        }
    
        Strafed = true;
        // Serial.println("obstacle avoid strafe right");
        // memory = this->direction;

      }else if(direction < -directionThresh){    //Strafe left
        this->avoidanceOn = true;
        if(!Strafed){
          startTime = millis();
          Serial.println("detect object right = " + String(RF_IR.isObject()));
          if(RF_IR.isObject()){
            Serial.println("sonar object = " + String(sonar.isObject()));
            Serial.println("LR IR = " + String(LR_IR.getReading()));
            if(sonar.isObject() && (LR_IR.getReading() < ObstacleSizeMax)){
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime/2);
            }else if(!sonar.isObject() && (LR_IR.getReading() < (ObstacleSizeMax/2))){
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime);
              Serial.println("Inversing direction - Only RF on object");
            }else{
              invDirection = false;
              wheels.Strafe(LEFT, 0);
              Serial.println("Keep direction - Only RF on object");
            }
          }else if (sonar.ReadUltraSonic() < 220) {
            if(LR_IR.getReading() < ObstacleSizeMax){
              invDirection = true;
              wheels.Strafe(RIGHT, momentumTime/2);
              Serial.println("Inversing direction - Both sonar and RF on object");
            }else{
              invDirection = false;
              wheels.Strafe(LEFT, 0);
              Serial.println("Keep direction - Both sonar and RF on object");
            }
        }else
          {
            wheels.Strafe(LEFT, 0);
          }
          
        }else{
          Strafed = true;
          Serial.println("obstacle avoid strafe left");
          memory = this->direction;
        }
        if(invDirection){
          memory = -(this->direction);
        }else{
          memory = this->direction;
        }
        

        Strafed = true;
        // Serial.println("obstacle avoid strafe left");
        // memory = this->direction;

      }else{                        //Going forward
        if(Strafed){                //On previous loop the car had strafed to avoid
          stopTime = millis();
          Strafed = false;
          passWait = true;
        }
        if(avoidanceOn){           //if it is in the process of avoiding an obstacle
          wheels.Straight(200);    //go straight
          Serial.println("no obsticle go straight");
        }
        else{
          Serial.println("OBSTACLE AVOID: deattaching wheel");
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
            delay(400);               //wait for back wheel to pass obstical
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
            delay(400);                                      //wait for back wheel to pass obstical
            wheels.Strafe(LEFT, (stopTime - startTime));    //Strafe back
            this->avoidanceOn = false;
            Serial.println("strafe back right");
          }
        }
      }
      //memory = direction;       //Store initial strafe direction (so we know where to strafe back)
    }
  }

  return avoidanceOn; //true means it is avoiding, false means it is not avoiding
}


float Robot::NEAR(float dist, bool isIR){    //Fuzzificaiton
  float thresh1,thresh2;
  if (isIR){
    thresh1 = IRNearThresh1;
    thresh2 = IRNearThresh2;
  }else{
    thresh1 = USNearThresh1;
    thresh2 = USNearThresh2;
  }
  
  float g = -1/(thresh2 - thresh1);
  float c = 1 + (-g)*thresh1;

  if(dist <= thresh1){
    //Serial.println("In NEAR: 1"); 
    return 1.0;
  }else if(dist < thresh2 && dist > thresh1){
    //Serial.println("In NEAR: " + String(g*dist + c)); 
    return (g*dist + c);
  }else{
    //Serial.println("In NEAR: 0"); 
    return 0.0;
  }
}

float Robot::FAR(float dist, bool isIR){     //Fuzzification
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
    //Serial.println("In FAR: 0"); 
    return 0.0;
  }else if(dist < thresh2 && dist > thresh1){
    //Serial.println("In FAR: " + String(g*dist + c));
    return (g*dist + c);
  }else{
    //Serial.println("In FAR: 1"); 
    return 1.0;
  }
}

float Robot::Left_Rules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF){
  //float A = min3(LeftN, CenterN, RightN);    rule taken away as it may give too much power to left
  float C = min3(LeftN, CenterF, RightN);
  float E = min3(LeftF, CenterN, RightN);
  //float F = min3(LeftF, CenterN, RightF);
  float G = min3(LeftF, CenterF, RightN);

  //float temp1 = max(A,C);
  float temp2 = max(E,G);
  return max(C, temp2);           //Return max of the same rules
  
}

float Robot::Right_Rules(float LeftN, float LeftF, float RightF, float CenterN, float CenterF){
  float B = min3(LeftN, CenterN, RightF);     
  float D = min3(LeftN, CenterF, RightF);
  float F = min3(LeftF, CenterN, RightF);
  
  float temp1 = max(B,D);
  return max(temp1,F);                //Return max of the same rules
}

float Robot::Forward_Rules(float LeftN, float RightN, float CenterN){ 
  float H = min3(LeftN, CenterN, RightN);
  return H;                       //Return max of the same rules
}

float Robot::min3(float a, float b, float c){
  if(a <= b && a <= c){
    return a;
  }else if(b <= c){
    return b;
  }else{
    return c;
  }
}

// Fuzzification combining distance readings from left, right and center. 
// float* Robot::fuzzify(float e){
//   static float fuzz[3];
//   int thresh1 = 6;
//   int thresh2 = 3;

//   if(e <= -thresh1 || e >= thresh1){
//     fuzz[0] = 0; // left
//     fuzz[1] = 0; //right
//     fuzz[2] = 1; //no obstacle
//   }else if(-thresh1 < e && e < -thresh2){
//     fuzz[0] = 0; // left
//     fuzz[1] = 2 + e/3; //right
//     fuzz[2] = -e/3 - 2; //no obstacle
//   }else if(-thresh2 <= e && e < 0){
//     fuzz[0] = 0; // left
//     fuzz[1] = 1; //right
//     fuzz[2] = 0; //no obstacle
//   }else if(0 <= e && e <= thresh2){
//     fuzz[0] = 1; // left
//     fuzz[1] = 0; //right
//     fuzz[2] = 0; //no obstacle
//   }else if(thresh2 < e && e < thresh1){
//     fuzz[0] = 0; // left
//     fuzz[1] = 2 - e/3; //right
//     fuzz[2] = e/3 + 2; //no obstacle
//   }
// }


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


// using fuzzy logic
// int Robot::go_target(){
//   //wheels.Move(0,80);
//   int tar_arrive = 0;
//    bool direction; 

//   while (tar_arrive != 1) {
//     Serial.println("Going ot target");
//     Serial.println("Abstacle avoid");
//     this->obstacle_Avoid();

//     float fuzzy_var = this->lightInfo->detect_dir();
//     Serial.print("fuzzy reading:  ");
//     Serial.println(fuzzy_var);
//     if (fuzzy_var <0){
//         direction = false;
//     }    


//     float speed = fuzzy_var * this->speed_step;
//     Serial.print("Speed:   ");
//     Serial.println(speed);
//     this->wheels.Turn(direction,speed);
//     delay(10); // allow rotation to happen
//     this->wheels.Straight(200);
    
//     // check if meet target
//     if ((this->sonar.ReadUltraSonic() < this->thr_sonar) && this->lightInfo->detect_front()){
//       Serial.println("Target found!");
//       tar_arrive = 1;
//     } 
//   }

//   // arrived at the target
//   // this->wheels.Stop();
//   // alternative:
//   // this->wheels.Disable(); 

//   return tar_arrive;
// }

// return false = haven't arrived to target
// return true = have arrived to target
bool Robot::go_target(){

  Serial.println("IN GO TARGET");

  bool dir; 
  float Kp = 0.15;
  float Ki = 0.005;
  float accumulation = 0;
  float distLC = 0;
  float distRC = 0;
  float error = 0;
  float RLerror = 0;
  int search;
  float speed, ultraSpeed;
  bool go_enable = false;

  //search = this->rotate_while_scan(true); // initial searching

  // float LLAve = this->lightInfo->PT_LL->getRawReading();
  // float LCAve = this->lightInfo->PT_LC->getRawReading();
  // float RCAve = this->lightInfo->PT_RC->getRawReading();
  // float RRAve = this->lightInfo->PT_RR->getRawReading();

  float LLAve = this->lightInfo->PT_LL->getRawReading();
  float LCAve = this->lightInfo->PT_LC->getRawReading();
  float RCAve = this->lightInfo->PT_RC->getRawReading();
  float RRAve = this->lightInfo->PT_RR->getRawReading();

  distLC = this->lightInfo->PT_LC->getDistance();
  distRC = this->lightInfo->PT_RC->getDistance();

  //when error is positive, right side is brighter hence turn right
  float centre_error = RCAve-LCAve;
  float end_error = RRAve-LLAve;
  error = RCAve+RRAve - LCAve-LLAve; 

  if ((RRAve > TARGET_BRIGHTNESS_OUT_R) || (LLAve > TARGET_BRIGHTNESS_OUT_L) || (((distLC + distRC)/2) > TARGET_DISTANCE)) {
    return true;
  } else {
    go_enable = true;
  }

  if (go_enable) {
    if ((((RCAve+LCAve)/2) < CENTRE_LOW) && (((RRAve+LLAve)/2) > END_HIGH) && (abs(centre_error) > CENTRE_ERROR)){
      if (centre_error > 0){
        dir = true;  // turn right
      } else{
        dir = false; // turn left
      }

      this->wheels.Turn(dir, 200);

    } else if ((((RRAve+LLAve)/2) < END_LOW) && (((RCAve+LCAve)/2) > CENTRE_HIGH) && (abs(end_error) > END_ERROR)){
      if (end_error > 0){
        dir = true;  // turn right
      } else{
        dir = false; // turn left
      }

      this->wheels.Turn(dir, 200);

    } else if (abs(error) > ABS_ERROR) {
      float start_time = millis();  
      while  ((abs(error) > ABS_ERROR) || ((millis()-startTime)<2000)) { 
      // Only accumulate error if it is small
      if(abs(error) < (ABS_ERROR + 100)){  
        accumulation = accumulation + error;
      }
      speed = Kp*error + Ki*accumulation;
      constrain(speed, -500 , 500);

      // Serial.println("Correcting direction..");
      // Serial.print("Current error:    "); 
      // Serial.println(error);
      // Serial.print("Speed turning:");
      // Serial.println(speed);

      // Determine the direction we are supposed to turn
      if (error > 0){
        // Positive error means right side is brighter
        dir = true;
      } else {
        // Negative error means left side is brighter
        dir = false;
      }

      Serial.println("turning the robot");
      this->wheels.Turn(dir, speed);
      LLAve = this->lightInfo->PT_LL->getRawReading();
      LCAve = this->lightInfo->PT_LC->getRawReading();
      RCAve = this->lightInfo->PT_RC->getRawReading();
      RRAve = this->lightInfo->PT_RR->getRawReading();
      error = RCAve+RRAve - LCAve-LLAve;
      // Serial.print("Updated error:    "); 
      // Serial.println(error);
      }
    }
  } else {
    this->wheels.Straight(200);
  }
  return false;
}

  // //---------------------OLD CODE------------------------------//


  // //Closer to the light, higher the ADC value
  // //We want to adjust the direction of the robot the robot is far from the light
  // if (  ((distLC+distRC)/2 > TARGET_DISTANCE) && ((LCAve +RCAve)/2 < TARGET_BRIGHTNESS) && (RRAve <TARGET_BRIGHTNESS_OUT) && (LLAve < TARGET_BRIGHTNESS_OUT)) { // distance from the centre phototransistor values
  //   // this->obstacle_Avoid();
    
  //   LLAve = this->lightInfo->PT_LL->getRawReading();
  //   LCAve = this->lightInfo->PT_LC->getRawReading();
  //   RCAve = this->lightInfo->PT_RC->getRawReading();
  //   RRAve = this->lightInfo->PT_RR->getRawReading();

  //   // LLAve = this->lightInfo->PT_LL->getAverageReading();
  //   // LCAve = this->lightInfo->PT_LC->getAverageReading();
  //   // RCAve = this->lightInfo->PT_RC->getAverageReading();
  //   // RRAve = this->lightInfo->PT_RR->getAverageReading();

  //   RLerror = RRAve - LLAve;

  //   // Commented out the check if light disappears, need to check if our robot will veer off for long distance
  //   // if (((RCAve+RRAve+LCAve+LLAve)/4) < 10){ // light disappear
  //   //   dir = true; // turn right
  //   //   search = this->rotate_while_scan(dir);
  //   // } 
    
  //   if ((((RCAve+RRAve+LCAve+LLAve)/4) < 60) && (((RCAve+RRAve+LCAve+LLAve)/4) > 30)){ // Fixes trajectory when light is somewhere but we might be off
  //     Serial.println("In go_target, the light is slightly off");
  //     // Rotate to the highest light value
  //     if (RRAve>=LLAve){
  //       dir = true;  // turn right
  //     } else{
  //       dir = false; // turn left
  //     }

  //     //while((((RCAve+RRAve+LCAve+LLAve)/4) < 60)) {
  //     while((((RCAve+LCAve)/2) < 30)) {
  //       Serial.println("In go_target, while loop to turn from slightly off");
  //       this->wheels.Turn(dir, 100);
  //       LLAve = this->lightInfo->PT_LL->getRawReading();
  //       LCAve = this->lightInfo->PT_LC->getRawReading();
  //       RCAve = this->lightInfo->PT_RC->getRawReading();
  //       RRAve = this->lightInfo->PT_RR->getRawReading();
  //     }
  //     //search = this->rotate_while_scan(dir);
  //   } 
  //   // else if ((abs(RLerror) < 35) && (RCAve < 15) && (LCAve < 15)) {   
  //   //   Serial.println("In go_target, the robot detects two lights far away");
  //   //   // Fixes problem when the robot is placed centre between two lights and starts driving straight to the centre
  //   //   // RR and LL reads very high at similar values and LC and RC reads low
  //   //   // Rotate to the highest light value
  //   //   if (RRAve>LLAve){
  //   //     dir = true;  // turn right
  //   //   }else{
  //   //     dir = false; // turn left
  //   //   }
  //   //   this->wheels.Turn(dir, 100); //turn the robot
  //   //   delay(5000);                  //give time for the robot to turn away
  //   // }

  //   float start_time = millis();    
  //   error = RCAve+RRAve - LCAve-LLAve; 
  //   // Change direction if the error is too higher or the time spent in the loop is less than 2 sec
  //   while  ((abs(error) > 300) || ((millis()-startTime)<2000)) { 
  //     // Only accumulate error if it is small
  //     if(abs(error) < 380){   // Previously we had a value of 20, but it would never enter the while loop with 20
  //       accumulation = accumulation + error;
  //     }
  //     speed = Kp*error + Ki*accumulation;
  //     constrain(speed, -500 , 500);

  //     // Serial.println("Correcting direction..");
  //     // Serial.print("Current error:    "); 
  //     // Serial.println(error);
  //     // Serial.print("Speed turning:");
  //     // Serial.println(speed);

  //     // Determine the direction we are supposed to turn
  //     if (error > 0){
  //       // Positive error means right side is brighter
  //       dir = true;
  //     } else {
  //       // Negative error means left side is brighter
  //       dir = false;
  //     }

  //     Serial.println("turning the robot");
  //     this->wheels.Turn(dir, speed);
  //     LLAve = this->lightInfo->PT_LL->getRawReading();
  //     LCAve = this->lightInfo->PT_LC->getRawReading();
  //     RCAve = this->lightInfo->PT_RC->getRawReading();
  //     RRAve = this->lightInfo->PT_RR->getRawReading();
  //     error = RCAve+RRAve - LCAve-LLAve;
  //     // Serial.print("Updated error:    "); 
  //     // Serial.println(error);
  //   }

  //   this->wheels.Straight(200);
  //   return false;

  // // Reached the light    
  // } else {
  //   // float sonarRead = this->sonar.ReadUltraSonic(); //read the ultrasonic sensor
  //   // float desDistance  = 100;                        //desired distance from the light
  //   // float distError = sonarRead - desDistance;      //if error positive we move closer otherwise we stay there

  //   // // Once the light is in range, we should use the ultrasonic to get closer to the light
  //   // while (distError > 15) {
  //   //   Serial.println("Adjusting distance from the light");
  //   //   ultraSpeed = distError*1.2;
  //   //   constrain(speed, -500 , 500);
  //   //   this->wheels.Straight(ultraSpeed);
  //   //   // Reread ultrasonic distance
  //   //   sonarRead = this->sonar.ReadUltraSonic();
  //   //   // Recalculate error
  //   //   distError = sonarRead - desDistance;
  //   // }

  //   //  if ((distLC+distRC)/2 <= 400) {
  //   //     this->stopPos = 1;
  //   //  } else if((LCAve +RCAve)/2 >= TARGET_BRIGHTNESS){
  //   //     this->stopPos = 1;
  //   //  } else if (RRAve >= TARGET_BRIGHTNESS_OUT) {
  //   //     this->stopPos = 2;
  //   //  } else if (LLAve >= TARGET_BRIGHTNESS_OUT) {
  //   //     this->stopPos = 3;
  //   //  }

  //   // Serial.println("Target arrived!");
  //   return true;
  // }
  //}

void Robot::servoLeft(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MAX);
  this->wheels.fanServo.write(70);
  
}

void Robot::servoRight(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MIN);
  this->wheels.fanServo.write(50);
}

void Robot::servoReset(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MIDDLE);
  this->wheels.fanServo.write(60);
}

void Robot::servoRotate(){
  float LLAve = this->lightInfo->PT_LL->getRawReading();
  float LCAve = this->lightInfo->PT_LC->getRawReading();
  float RCAve = this->lightInfo->PT_RC->getRawReading();
  float RRAve = this->lightInfo->PT_RR->getRawReading();
  float angle = SERVO_MIDDLE;

  float centreError = RCAve - LCAve;
  //continue to rotate the servo until LC and RC have a high value
  while ((abs(centreError) > 100) || (LCAve < SERVO_TARGET_BRIGHTNESS) || (RCAve < SERVO_TARGET_BRIGHTNESS)) {
    // Serial.println("Right photo: " + String(RRAve) + " Left photo: " +  String(LLAve));
    // Serial.println("Right Centre photo: " + String(RCAve) + " Left Centre photo: " +  String(LCAve));
    // Serial.println("Centre error: " + String(centreError));
    // Rotate to the highest light value
    if (RRAve>LLAve || centreError > 0){
      angle = angle - 10; // turn right
      // Serial.println("Turn right, decrease angle");
    } else{
      angle = angle + 10; // turn left
      // Serial.println("Turn left, increase angle");
    }

    constrain(angle, SERVO_MIN , SERVO_MAX);
    // Serial.println("Angle: " + String(angle));
    this->wheels.fanServo.write(angle);

    LLAve = this->lightInfo->PT_LL->getRawReading();
    LCAve = this->lightInfo->PT_LC->getRawReading();
    RCAve = this->lightInfo->PT_RC->getRawReading();
    RRAve = this->lightInfo->PT_RR->getRawReading();
    centreError = RCAve - LCAve;

    delay(50);
  }
  // Serial.println("Light reached");

}
