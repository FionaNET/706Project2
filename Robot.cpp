#include "Robot.h"
#include <Arduino.h>
#include <math.h>


#define obstacleThresh 150

Robot::Robot(){
    this->LF_IR = IR_Sensor(LONG,IR_LF);
    this->RF_IR = IR_Sensor(LONG,IR_RF);
    this->LR_IR = IR_Sensor(MID,IR_LR);
    this->RR_IR = IR_Sensor(MID,IR_RR);

    this->wheels = RobotBase();

    this->gyro = Gyroscope();
    this->sonar = UltrasonicSensor();
    this->PassFlagOn = false;
    this->Strafed = false;
    this->thr_sonar = 10; // need to adjust
    this->speed_step = 500;
}

//return 1 = successfully detected light
//return 2 = stopped due to timeout
int Robot::rotate_while_scan(){

    //bool front = lightInfo->detect_front();
    float timeStart = millis();
    float timeout = 10000; // 10 sec for a full rotation, need calibration
    bool front = false;
    while (!front) {
        Serial.println("Rotate while scanning loop");
        this->wheels.Turn(true, 200); // trun right 360 deg and scan
        front = lightInfo->detect_front();
        if (millis()-timeStart > timeout) {
          Serial.println("10 seconds, STOP");
            return 2;
        }
    }
    Serial.println("Found light, should go target");
    return 1;

    //this->wheels.Stop();
    // alternative:
    //this->wheels.Disable();  

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

//obstical avoidance with fuzzy logic
void Robot::obstacle_Avoid(){
  //int far_thresh = 10;
  //int close_thresh = 3;
  float d1, d2, d3;

  Serial.println("LF_IR dist: " + String(LF_IR.getReading()) + "RF_IR dist: " + String(RF_IR.getReading()) + "Sonar dist: " + String(sonar.ReadUltraSonic()));

  Serial.println("  LF OBject: " + String(LF_IR.isObject()) + "  RF object: " + String(LR_IR.isObject()) + "  center object: " + String(sonar.isObject()));
  if(LF_IR.isObject() && RF_IR.isObject() && sonar.isObject()){
    //All three sensors are reading objects so it is a wall
    this->CL_Turn(90);
    Serial.println("There is a wall so we are turning");

  }else {
    //1 or more objects detected so it is a cyclindar
    //get all distance readings

    if(!PassFlagOn){      //Only do this once when no obsticles have been previously detected
      d1 = LF_IR.getReading();
      d2 = RF_IR.getReading();

      d3 = sonar.ReadUltraSonic();
      Serial.println("D1 = " + String(d1) + "  D2 = " + String(d2) + "  D3 = " + String(d3));

      float LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      float RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      float ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));
      Serial.println("LeftMax = " + String(LeftMax) + "  RightMax = " + String(RightMax) + "  ForwardMax = " + String(ForwardMax));

      //take weighted average
      this->direction = LeftMax*-50 + ForwardMax*10 + RightMax*50;    //get direction
      Serial.println("Direction = " + String(direction));
    }
    
    if (direction > 25){          //Strafe right
      if(!Strafed){
        startTime = millis();     //Count how long we have strafed for
        memory = direction;       //Store initial strafe direction (so we know where to strafe back)
      }
      wheels.Strafe(LEFT, 0);
      delay(200);
      Strafed = true;
      Serial.println("obsticle avoid strafe right");

    }else if(direction < -25){    //Strafe left
      if(!Strafed){
        startTime = millis();
        memory = direction;     //Store initial strafe direction (so we know where to strafe back)
      }
      wheels.Strafe(RIGHT, 0);
      delay(200);
      Strafed = true;
      Serial.println("obsticle avoid strafe left");

    }else{
      if(Strafed){                //On previous loop the car had strafed
        stopTime = millis();
        Strafed = false;
        PassFlagOn = true;
      }
      wheels.Straight(200);    
      Serial.println("no obsticle go straight");
    }

    if(PassFlagOn){
      //turn flag off once the back IR sensors read the obstical (meaning we have passed it)
      if(memory < 0){   //Strafed left at start
        PassFlagOn = !(RR_IR.getReading() < obstacleThresh);
        if(!PassFlagOn){
          delay(400);
          wheels.Strafe(LEFT, (stopTime - startTime));
          Serial.println("strafe back right"); 
        }
      }else
      {
        PassFlagOn = !(LR_IR.getReading() < obstacleThresh);
        if(!PassFlagOn){  //Once obstical has passed
          delay(400);
          wheels.Strafe(RIGHT, (stopTime - startTime));    //Strafe back
          Serial.println("strafe back left");
        }
      }
    }
  }
}

    float Robot::NEAR(float dist, bool isIR){    //Fuzzificaiton
      float thresh1,thresh2;
      if (isIR){
        thresh1 = 100;
        thresh2 = 200;
      }else{
        thresh1 = 70;
        thresh2 = 140;
      }
      
      float g = -1/(thresh2 - thresh1);
      float c = 1 + g*thresh1;

      if(dist <= thresh1){
        return 1.0;
      }else if(dist < thresh2 && dist > thresh1){
        return (g*dist + c);
      }else{
        return 0.0;
      }
    }

    float Robot::FAR(float dist, bool isIR){     //Fuzzification
      float thresh1,thresh2;
      if (isIR){
        thresh1 = 100;
        thresh2 = 200;
      }else{
        thresh1 = 100;
        thresh2 = 200;
      }
      float g = 1/(thresh2 - thresh1);
      float c = -g*thresh1;

      if(dist <= thresh1){
        return 0.0;
      }else if(dist < thresh2 && dist > thresh1){
        return (g*dist + c);
      }else{
        return 1.0;
      }
    }

    float Robot::Left_Rules(float LeftN, float LeftF, float RightN, float RightF, float CenterN, float CenterF){
      float A = min3(LeftN, CenterN, RightN);
      float C = min3(LeftN, CenterF, RightN);
      float E = min3(LeftF, CenterN, RightN);
      //float F = min3(LeftF, CenterN, RightF);
      float G = min3(LeftF, CenterF, RightN);

      float temp1 = max(A,C);
      float temp2 = max(E,G);
      return max(temp1, temp2);           //Return max of the same rules
      
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
    //     fuzz[2] = 1; //no obstical 
    //   }else if(-thresh1 < e && e < -thresh2){
    //     fuzz[0] = 0; // left
    //     fuzz[1] = 2 + e/3; //right
    //     fuzz[2] = -e/3 - 2; //no obstical
    //   }else if(-thresh2 <= e && e < 0){
    //     fuzz[0] = 0; // left
    //     fuzz[1] = 1; //right
    //     fuzz[2] = 0; //no obstical
    //   }else if(0 <= e && e <= thresh2){
    //     fuzz[0] = 1; // left
    //     fuzz[1] = 0; //right
    //     fuzz[2] = 0; //no obstical
    //   }else if(thresh2 < e && e < thresh1){
    //     fuzz[0] = 0; // left
    //     fuzz[1] = 2 - e/3; //right
    //     fuzz[2] = e/3 + 2; //no obstical
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

    while (abs(angle_error) > angle_thres && turn_time <= 2000) {       //will exit while loop if the time in the while loop exceeds 2sec
    
    if(abs(angle_error) < 12.0){
      cumulation = cumulation + angle_error;
    }

    //noticed the robot had a problem where at the start of turning, it would think it'll need to turn CCW due to a negative angle error
    //since the robot is always turning CW, we absolute the rot_change for large angle_error 
    if(abs(angle_error) > 80){
      rot_change = abs(kp_angle * angle_error + ki_angle*cumulation);
    }else{
      rot_change = kp_angle * angle_error + ki_angle*cumulation;
    }

    rot_change = constrain(rot_change, -500, 500);
    
    Serial.print("The rot_change is: ");
    Serial.println(rot_change);
    
    //set the motors
    if(rot_change < 0){
      wheels.Turn(CCW, abs(rot_change));
    }else
    {
      wheels.Turn(CW, rot_change);
    }

    //read the gyro sensor and get current angle
    angle_error = ref_angle - gyro.GyroRead();
    Serial.print("The angle error is: ");
    Serial.println(angle_error);
    turn_time = millis() - turn_start;
  }
  }


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

    bool Robot::go_target(){

      int tar_arrive = 0;
      float Kp = 5;

      while (tar_arrive != 1){

        float LLAve = this->lightInfo->PT_LL->getAverageReading();
        float LCAve = this->lightInfo->PT_LC->getAverageReading();
        float RCAve = this->lightInfo->PT_RC->getAverageReading();
        float RRAve = this->lightInfo->PT_RR->getAverageReading();
        

        if (((RCAve+RRAve+LCAve+LLAve)/4) <500) {
          this->obstacle_Avoid();
          
          LLAve = this->lightInfo->PT_LL->getAverageReading();
          LCAve = this->lightInfo->PT_LC->getAverageReading();
          RCAve = this->lightInfo->PT_RC->getAverageReading();
          RRAve = this->lightInfo->PT_RR->getAverageReading();

          if (((RCAve+RRAve+LCAve+LLAve)/4) < 10){ // if light disappear
            this->rotate_while_scan();
          } else {
            float error = RCAve+RRAve - LCAve-LLAve;  
            while  (error > 50) {
              float speed = Kp*error;
              bool dir;
              if (error>0){
                dir = true;
              } else {
                  dir = false;
              }
              this->wheels.Turn(dir, speed);
              LLAve = this->lightInfo->PT_LL->getAverageReading();
              LCAve = this->lightInfo->PT_LC->getAverageReading();
              RCAve = this->lightInfo->PT_RC->getAverageReading();
              RRAve = this->lightInfo->PT_RR->getAverageReading();
              error = RCAve+RRAve - LCAve-LLAve;
            }

             this->wheels.Straight(200);
            
          }

         
        } else {
          Serial.println("Target arrived!");
          tar_arrive = 1;
          return  true;
        }

      }
  
    }

