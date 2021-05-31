#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "RobotBase.h"
#include "Robot.h"

Firefighting fighter = Firefighting(15);
LightDetect lightInfo = LightDetect();
Robot robot = Robot();
Phototransistor LL = Phototransistor(PHOTOTRANSISTOR1);
Phototransistor LC = Phototransistor(PHOTOTRANSISTOR2);
Phototransistor RC = Phototransistor(PHOTOTRANSISTOR3);
Phototransistor RR = Phototransistor(PHOTOTRANSISTOR4);

void setup(){
  Serial.begin(9600);
  Serial.println("Setting up");
  
  robot.wheels.Attach();           //must call within setup or loop to attach the wheels
  robot.gyro.GyroscopeCalibrate(); //call to remove the bias from gyroscope
  //robot.gyro.currentAngle = 0;
  delay(1000);
  Serial.println("Start main loop");
}

int search = 0;               //variable to determine if we need to search for a light
bool target_reached = false;  //variable to store if the light has been reached
//bool start = true;            //lets us know we are at the beginning of the run and will reset after every fire has been blow out
float time_start = 0;         //store the time at which we finised the rotate_while_scan
int fire = 0;                 //variable to keep track of how many fires to blow out
int state = 1;
bool fireOff;
float start;
void loop(){

switch (state) {
  
  case 1:
  Serial.println("In case 1, searching the light");
     // initial state:
    search = robot.rotate_while_scan(true);

    if (search == 1){ // light found!!
      state = 2; 
    } else {
      state = 4;
    }
    break;
  case 2:
  Serial.println("In case 2, go to light");
    // find the going to target normal routine
//    robot.obstacle_Avoid();
    target_reached = robot.go_target();

    if (target_reached){
      robot.wheels.Disable();
      delay(1000); //give time for the motors to stop before turning on fan
      state = 3;
    }
    break;
  case 3:
  Serial.println("In case 3, firefight");
   // firefighting state
    fireOff = fighter.ExtinguishFire();
    if (fireOff){
      fire = fire +1;
      delay(1000); //give time for the fan to fully turn off
      robot.wheels.Attach();
      //reinitialise so we can extinguish the next fire
      fighter.Fire_extinguish = 0;
      if (fire < 2) {
           //reverse
           robot.wheels.Straight(-200);
           delay(200);
           state = 1;
      }else{
        state = 5;
      }
          
    }
    break;
    
    case 4:
    Serial.println("In case 4, go straight");
      // go straight 

      start = millis();
      while ((millis()-start)<1500){
        robot.obstacle_Avoid();
        robot.wheels.Straight(200);  
      }
      state = 1; 
      break;

    case 5:
    Serial.println("In case 5, STOP");
      robot.wheels.Disable();
      break;
}
}

  
//  //Serial.print("READING ULTRASONIC: ");
//  //Serial.println(robot.sonar.ReadUltraSonic());
//  
//  //blowing out 2 fires take highest priority
//  if (fire == 2) {
//    robot.wheels.Disable();
//
//  //blowing out the fire is second priority because it might trigger obstacle avoidance
//  } else if (target_reached) {
//     robot.wheels.Disable();
//     delay(1000); //give time for the motors to stop before turning on fan
//
//     //extinguish the fire
//     if (fighter.ExtinguishFire()) {
//      delay(1000); //give time for the fan to fully turn off
//      robot.wheels.Attach();
//
//      //reinitialise so we can extinguish the next fire
//      fighter.Fire_extinguish = 0;
//
//      //search again
//      start = true;
//      search = 0;
//
//      //find the next target
//      target_reached = false;
//      
//      //increase the number of fires we put out
//      fire = fire + 1;
//
//      if (fire < 2) {
//      //reverse
//      robot.wheels.Straight(-200);
//      delay(200);
//      }
//     }
//     
//  // Based on if any of the range sensors detect an object
//  }else if (robot.check() != 0) {
////    while( !robot.obstacle_Avoid()) {
////      robot.obstacle_Avoid();
////    }
//    robot.obstacle_Avoid();
//  }else if (search != 1) {
//    //rotate at the start or after 1.5 second of going straight
//    if ((start == true) || (millis()-time_start) > 1500){
//      search = robot.rotate_while_scan(true);
//      time_start = millis(); 
//      start = false;
//    }
//    //go straight for 1.5 sec
//    robot.wheels.Straight(200);
//  } else {
//    //by default the robot should go to target
//    target_reached = robot.go_target();
//  }
//
//
//}
