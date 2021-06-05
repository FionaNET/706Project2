#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "RobotBase.h"
#include "Robot.h"

Firefighting fighter = Firefighting(15);
Robot robot = Robot();

void setup(){
  Serial.begin(9600);
  Serial.println("Setting up");
  
  robot.wheels.Attach();            //must call within setup or loop to attach the wheels
  robot.gyro.GyroscopeCalibrate();  //call to remove the bias from gyroscope

  fighter.FanServoAttach();
  fighter.Servo_Reset();
  
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
    
    search = robot.Rotate_While_Scan(true);
    
    if (search == 1){    //Light found!!
      state = 2;         //Go target state
    } else {
      state = 4;         //Go straight if light is not found
    }
    
    break;
    
  case 2:
    Serial.println("In case 2, go to light");

    if (robot.Obstacle_Avoid()) {   //true means it is avoiding something
      robot.Obstacle_Avoid();
    } else {
      robot.wheels.Attach();
      target_reached = robot.Go_Target();
    }
    
    if (target_reached){
      robot.wheels.Disable();
      delay(1000); //give time for the motors to stop before turning on fan
      state = 3;
    }
    
    break;
    
  case 3:
    Serial.println("In case 3, firefight");
    
    fireOff = fighter.Servo_Rotate();
    if (fireOff){
      
      fire = fire + 1;
      delay(1000);                    //Give time for the fan to fully turn off
      robot.wheels.Attach();          //Reattach robot wheels
      fighter.Fire_extinguish = 0;    //Reinitialise so we can extinguish the next fire
      fighter.Servo_Reset();           //Reset servo position
      if (fire < 2) {
         //reverse
         robot.wheels.Straight(-200);
         delay(300);
         state = 1;                   //Go back to searching again
      } else {
        state = 5;                    //Stop the motors when 2 fires have been blown
      }
          
    }
    
    break;
    
    case 4:
      //Go straight when searching failed
      Serial.println("In case 4, go straight");
  
      start = millis();
      //Go straight for 1.5 seconds
      while ((millis()-start) < 1500){
        //Only obstacle avoid if it does not pass the check
        //robot.obstacle_Avoid();
        
        if (robot.Obstacle_Avoid()) {   //true means it is avoiding something
          robot.Obstacle_Avoid();
        } else {
          robot.wheels.Attach();
          robot.wheels.Straight(150);  
        }
        //robot.wheels.Straight(200);  
      }
      
      state = 1; //Go back to search again
      break;

    case 5:
      //Stop the motors once all fires are blown out
      Serial.println("In case 5, STOP");
      robot.wheels.Disable();
      fighter.FanServoDisable();
      break;
  }

}
