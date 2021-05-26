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

    this->gyro = Gyroscope();
    this->sonar = UltrasonicSensor();
    this->passWait = false;
    this->Strafed = false;
    this->thr_sonar = 10; // need to adjust
    this->speed_step = 500;

    this->avoidanceOn = false;
    this->invDirection = false;
}

//return 1 = successfully detected light
//return 2 = stopped due to timeout
int Robot::rotate_while_scan(bool dir){
  // search the light

  int speed;
  if (dir){ // true, turn right, CW
    speed = 200;
  }else{
    // false, turn left, CCW
    speed = -200;
  }

  Serial.println("Rotating while scan...");
    //bool front = lightInfo->detect_front();
    float timeStart = millis();
    float timeout = 5000; // 5 sec for a full rotation, need calibration
    bool front = false;
    while (!front) {
        Serial.println("Rotate while scanning loop");
        this->wheels.Turn(true, speed); // trun right 360 deg and scan
        front = lightInfo->detect_front();
        if (millis()-timeStart > timeout) {
          // Serial.println("10 seconds, STOP");

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

  float d1, d2, d3, d4, d5, LeftMax, ForwardMax, RightMax;


  Serial.println("LF_IR dist: " + String(LF_IR.getReading()) + "RF_IR dist: " + String(RF_IR.getReading()) + "Sonar dist: " + String(sonar.ReadUltraSonic()));
  Serial.println("  LF OBject: " + String(LF_IR.isObject()) + "  RF object: " + String(RF_IR.isObject()) + "  center object: " + String(sonar.isObject()));
  
  if(LF_IR.isObject() && RF_IR.isObject() && sonar.isObject()){
    //All three sensors are reading objects so it is a wall
    this->CL_Turn(90);
    Serial.println("There is a wall in front so we are turning");

  }else if(LF_IR.isObject() && LR_IR.isObject()) {
    this->CL_Turn(45);
    Serial.println("There is a wall on left so we are turning");
  }else if(RF_IR.isObject() && (RR_IR.isObject())){
    this->CL_Turn(-45);
    Serial.println("There is a wall on right so we are turning");
  }
  else {
    //1 or more objects detected so it is a cyclindar
    //get all distance readings

    //if(!this->passWait){      //Only do this once when no obsticles have been previously detected
      d1 = LF_IR.getReading();
      d2 = RF_IR.getReading();

      d3 = sonar.ReadUltraSonic();
      Serial.println("D1 = " + String(d1) + "  D2 = " + String(d2) + "  D3 = " + String(d3));

      LeftMax = Left_Rules(NEAR(d1, true), FAR(d1, true), NEAR(d2, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      RightMax = Right_Rules(NEAR(d1, true), FAR(d1, true), FAR(d2, true), NEAR(d3, false), FAR(d3, false));
      ForwardMax = Forward_Rules(NEAR(d1, true), NEAR(d2, true), NEAR(d3, false));
      Serial.println("LeftMax = " + String(LeftMax) + "  RightMax = " + String(RightMax) + "  ForwardMax = " + String(ForwardMax));

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
    if(direction > directionThresh){                          //Strafe right
      avoidanceOn = true;           
      if(!Strafed){              //Went straight last time or strafed in opposite direction
        startTime = millis();     //Count how long we have strafed for
        Serial.println("detect object left = " + String(LF_IR.isObject()));
        if(LF_IR.isObject()){
          Serial.println("sonar object = " + String(sonar.isObject()));
          Serial.println("RR IR = " + String(RR_IR.getReading()));
          if(sonar.isObject() && (RR_IR.getReading() < ObstacleSizeMax)){
            invDirection = true;
            wheels.Strafe(LEFT, 100);
          }else if(!sonar.isObject() && (RR_IR.getReading() < (ObstacleSizeMax))){
            invDirection = true;
            wheels.Strafe(LEFT, 200);
          }else{
            invDirection = false;
            wheels.Strafe(RIGHT, 0);
          }
       }else if (sonar.isObject())
       {
          if((RR_IR.getReading() < (200))){
            invDirection = true;
            wheels.Strafe(LEFT, 200);
          }else{
            invDirection = false;
            wheels.Strafe(RIGHT, 0);
          }
       }else{
          wheels.Strafe(RIGHT, 0);
       }
      }//else if(memory < -directionThresh){
      //   wheels.Strafe(LEFT, (millis() - startTime + 100));  //Strafe back and give momentum to strafe left
      //   startTime = millis() - 100;     //Count how long we have strafed for
      // }

      
  
      Strafed = true;
      Serial.println("obstacle avoid strafe left");
      memory = this->direction;

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
            wheels.Strafe(RIGHT, 100);
          }else if(!sonar.isObject() && (LR_IR.getReading() < (ObstacleSizeMax/2))){
            invDirection = true;
            wheels.Strafe(RIGHT, 200);
          }else{
            invDirection = false;
            wheels.Strafe(LEFT, 0);
          }
        }else if (sonar.isObject()) {
          if((LR_IR.getReading() < 200)){
            invDirection = true;
            wheels.Strafe(RIGHT, 200);
          }else{
            invDirection = false;
            wheels.Strafe(LEFT, 0);
          }
       }else
        {
          wheels.Strafe(LEFT, 0);
        }
        
      }//else if(memory > directionThresh){
      //   wheels.Strafe(RIGHT, (millis() - startTime + 100));  //Strafe back and give momentum to strafe right
      //   startTime = millis() - 100;     //Count how long we have strafed for
      // }
      

      Strafed = true;
      Serial.println("obstacle avoid strafe left");
      memory = this->direction;

    }else{                        //Going forward
      if(Strafed){                //On previous loop the car had strafed to avoid
        stopTime = millis();
        Strafed = false;
        passWait = true;
      }
      wheels.Straight(200);    
      Serial.println("no obsticle go straight");
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
        passWait = !(RR_IR.getReading() < obstacleThresh);
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

    float Robot::NEAR(float dist, bool isIR){    //Fuzzificaiton
      float thresh1,thresh2;
      if (isIR){
        thresh1 = 100;
        thresh2 = 200;
      }else{
        thresh1 = 120;
        thresh2 = 220;
      }
      
      float g = -1/(thresh2 - thresh1);
      float c = 1 + (-g)*thresh1;

      if(dist <= thresh1){
        Serial.println("In NEAR: 1"); 
        return 1.0;
      }else if(dist < thresh2 && dist > thresh1){
        Serial.println("In NEAR: " + String(g*dist + c)); 
        return (g*dist + c);
      }else{
        Serial.println("In NEAR: 0"); 
        return 0.0;
      }
    }

    float Robot::FAR(float dist, bool isIR){     //Fuzzification
      float thresh1,thresh2;
      if (isIR){
        thresh1 = 100;
        thresh2 = 200;
      }else{
        thresh1 = 120;
        thresh2 = 220;
      }
      float g = 1/(thresh2 - thresh1);
      float c = (-g)*thresh1;

      if(dist <= thresh1){
        Serial.println("In FAR: 0"); 
        return 0.0;
      }else if(dist < thresh2 && dist > thresh1){
        Serial.println("In FAR: " + String(g*dist + c));
        return (g*dist + c);
      }else{
        Serial.println("In FAR: 1"); 
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

    while ((abs(angle_error) > angle_thres) && (turn_time <= 2000)) {       //will exit while loop if the time in the while loop exceeds 2sec
    
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
    
    // Serial.print("The rot_change is: ");
    // Serial.println(rot_change);
    
    //set the motors
    if(rot_change < 0){
      wheels.Turn(CCW, abs(rot_change));
    }else
    {
      wheels.Turn(CW, rot_change);
    }

    //read the gyro sensor and get current angle
    angle_error = ref_angle - gyro.GyroRead();
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

    bool Robot::go_target(){
      bool dir; 
      int tar_arrive = 0;
      float Kp = 0.15;
      float Ki = 0.01;
      float accumulation = 0;
      float distLC = 0;
      float distRC = 0;
      int search;

      //search = this->rotate_while_scan(true); // initial searhing
      while (tar_arrive != 1){
        Serial.println("Target not found, going in the loop..."); 
        float LLAve = this->lightInfo->PT_LL->getRawReading();
        float LCAve = this->lightInfo->PT_LC->getRawReading();
        float RCAve = this->lightInfo->PT_RC->getRawReading();
        float RRAve = this->lightInfo->PT_RR->getRawReading();
        
        // Serial.print("LLAve: ");
        // Serial.print(LLAve);
        // Serial.print("  LCAve: ");
        // Serial.print(LCAve);
        // Serial.print("  RCAve: ");
        // Serial.print(RCAve);
        // Serial.print("  RRAve: ");
        // Serial.println(RRAve);

        // Serial.print("Average of the middle 2 phototransistors:    ");
        // Serial.println((RCAve+LCAve)/2);

        distLC = this->lightInfo->PT_LC->getDistance();
        distRC = this->lightInfo->PT_RC->getDistance();

        if ( (distLC+distRC)/2 > 275 ) { // distance 
         // this->obstacle_Avoid();
          
          LLAve = this->lightInfo->PT_LL->getRawReading();
          LCAve = this->lightInfo->PT_LC->getRawReading();
          RCAve = this->lightInfo->PT_RC->getRawReading();
          RRAve = this->lightInfo->PT_RR->getRawReading();

          if (((RCAve+RRAve+LCAve+LLAve)/4) < 10){ // if light disappear
            
            dir = true; // turn right
            search = this->rotate_while_scan(dir);
          }
          
          else if (((RCAve+RRAve+LCAve+LLAve)/4) < 30){ // light is somewhere but we might be very off
            
            if (RRAve>LLAve){
                dir = true; // turn right
            } else{
              dir = false; // turn left
            }
  
            search = this->rotate_while_scan(dir);
          }
            
            float error = RCAve+RRAve - LCAve-LLAve; 

            float start_time = millis();    
            while  (abs(error) > 350 || ((millis()-startTime)<2000)) {
              if(abs(error) < 20){
                   accumulation = accumulation + error;
              }
              // Serial.println("Correcting direction..");
              // Serial.print("Current error:    "); 
              // Serial.println(error);
              float speed = Kp*error + Ki*accumulation;
              
              // Serial.println(speed);
              constrain(speed, -500 , 500);
              // Serial.print("Speed turning:");
              // Serial.println(speed);

              bool dir;
              if (error>0){
                dir = true;
              } else {
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

             this->wheels.Straight(200);
            
          

         
        } else {
          Serial.println("Target arrived!");
          tar_arrive = 1;
          return  true;
        }

      }
  
    }

