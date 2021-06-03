#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "RobotBase.h"
#include "Robot.h"

Firefighting fighter = Firefighting(15);
Robot robot = Robot();
//LightDetect lightInfo = LightDetect();
//Phototransistor LL = Phototransistor(PHOTOTRANSISTOR1);
//Phototransistor LC = Phototransistor(PHOTOTRANSISTOR2);
//Phototransistor RC = Phototransistor(PHOTOTRANSISTOR3);
//Phototransistor RR = Phototransistor(PHOTOTRANSISTOR4);

void setup(){
  Serial.begin(9600);
  Serial.println("Setting up");
  
  robot.wheels.Attach();            //must call within setup or loop to attach the wheels
  robot.gyro.GyroscopeCalibrate();  //call to remove the bias from gyroscope
//  robot.fanServo.attach(SERVO_PIN);
//  robot.servoReset();
  
//  robot.fanServo.detach();
  //robot.FanServoDisable();
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
     // initial state: Seached for the light
    search = robot.rotate_while_scan(true);

    if (search == 1){    //Light found!!
      state = 2;         //Go target state
    } else {
      state = 4;         //Go straight if light is not found
    }
    break;
    
  case 2:
  Serial.println("In case 2, go to light");
    //Find the going to target normal routine
//    if (robot.obstacle_Avoid()) {
//      state = 1;
//      break;
//    }

    if (robot.obstacle_Avoid()) {
      
      robot.obstacle_Avoid();
    } else {
      Serial.println("CASE 2: reattaching wheel");
      robot.wheels.Attach();
      target_reached = robot.go_target();
    }
    
    if (target_reached){
      robot.wheels.Disable();
      delay(1000); //give time for the motors to stop before turning on fan
      state = 3;
    }
    
    break;
    
  case 3:
    Serial.println("In case 3, firefight");
    //rotate to the light
    robot.servoRotate();
    //Firefighting state
    fireOff = fighter.ExtinguishFire();
    if (fireOff){
      fire = fire + 1;
      delay(1000);                    //Give time for the fan to fully turn off
      robot.wheels.Attach();
      fighter.Fire_extinguish = 0;    //Reinitialise so we can extinguish the next fire
      robot.servoReset();     
      if (fire < 2) {
         //reverse
         robot.wheels.Straight(-200);
         delay(150);
         state = 1;                   //Go back to searching again
      }else{
        state = 5;                    //Stop the motors when 2 fires have been blown
      }
          
    }
    break;
    
    case 4:
    //change threshhold if the search failed then go back to state 1
    
    //Go straight when searching failed
    Serial.println("In case 4, go straight");

    start = millis();
    //Go straight for 1.5 seconds
    while ((millis()-start) < 1500){
      //Only obstacle avoid if it does not pass the check
      //robot.obstacle_Avoid();
      if (robot.obstacle_Avoid()) {
        state = 1;
        break;
      } 
      //robot.wheels.Straight(200);  
    }
    
    state = 1; //Go back to search again
    break;

    case 5:
    //Stop the motors once all fires are blown out
    Serial.println("In case 5, STOP");
    robot.wheels.Disable();
    robot.wheels.FanServoDisable();
    break;
}

}

  

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
