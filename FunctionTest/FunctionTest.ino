#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "RobotBase.h"
#include "Robot.h"

Firefighting fighter = Firefighting(15);
Robot robot = Robot();
//LightDetect lightInfo = LightDetect();
Phototransistor LL = Phototransistor(PHOTOTRANSISTOR1);
Phototransistor LC = Phototransistor(PHOTOTRANSISTOR2);
Phototransistor RC = Phototransistor(PHOTOTRANSISTOR3);
Phototransistor RR = Phototransistor(PHOTOTRANSISTOR4);

void setup(){
  Serial.begin(9600);
  Serial.println("Setting up");
  
  robot.wheels.Attach();           //must call within setup or loop to attach the wheels
  robot.gyro.GyroscopeCalibrate(); //call to remove the bias from gyroscope

  robot.wheels.FanServoAttach();
  robot.servoReset();
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
  robot.obstacle_Avoid();
  //Serial.print("READING ULTRASONIC: ");
  //Serial.println(robot.sonar.ReadUltraSonic());

  //Test obstacle avoidance when obstacle avoidance is a while loop
  //robot.obstacle_Avoid();
  //robot.wheels.Straight(200);
  
//  if (robot.check() != 0) {
//    robot.obstacle_Avoid();
//  } else {
//    robot.wheels.Straight(200);
//  }

  //Test obstacle avoidance when obstacle avoidance is NOT a while loop
//  if (robot.check() != 0) {
//    while( !robot.obstacle_Avoid()) {
//      robot.obstacle_Avoid();
//    }
//  } else {
//    robot.wheels.Straight(200);
//  }

//   //Test go target without obstacle avoidance
//   switch (state) {
//  
//    case 1:
//      Serial.println("In case 1, searching the light");
//       // initial state: Seached for the light
//      search = robot.rotate_while_scan(true);
//  
//      if (search == 1){    //Light found!!
//        state = 2;         //Go target state
//      } else {
//        state = 4;         //Go straight if light is not found
//      }
//      break;
//      
//    case 2:
//      Serial.println("In case 2, go to light");
//      //Find the going to target normal routine
//      //robot.obstacle_Avoid();
//      target_reached = robot.go_target();
//  
//      if (target_reached){
//        robot.wheels.Disable();
//        delay(1000); //give time for the motors to stop before turning on fan
//        state = 3;
//      }
//      
//      break;
//      
//    case 3:
//      Serial.println("In case 3, firefight");
//      //Firefighting state
//      robot.servoRotate();
//      fireOff = fighter.ExtinguishFire();
//      if (fireOff){
//        fire = fire + 1;
//        delay(1000);                    //Give time for the fan to fully turn off
//        robot.wheels.Attach();
//        fighter.Fire_extinguish = 0;    //Reinitialise so we can extinguish the next fire
//        robot.servoReset();
//        
//        if (fire < 2) {
//           //reverse
//           robot.wheels.Straight(-200);
//           delay(150);
//           state = 1;                   //Go back to searching again
//        }else{
//          state = 5;                    //Stop the motors when 2 fires have been blown
//        }
//            
//      }
//      break;
//      
//    case 4:
//      //Go straight when searching failed
//      Serial.println("In case 4, go straight");
//  
//      start = millis();
//      //Go straight for 1.5 seconds
//      while ((millis()-start) < 1500){   
//        robot.wheels.Straight(200);  
//      }
//      state = 1; //Go back to search again
//      break;
//  
//    case 5:
//      //Stop the motors once all fires are blown out
//      Serial.println("In case 5, STOP");
//      robot.wheels.Disable();
//      break;
//  }


//    robot.servoRotate();
//    fireOff = fighter.ExtinguishFire();
    
 

//  search = robot.rotate_while_scan();
//  while (search != 1){
//    // search the light at current position,
//    // if light is not found, go straight and re-search
// 
//    robot.wheels.Straight(200);
//    delay(1000);
//    search = robot.rotate_while_scan();
//  }
//  
//  if (robot.go_target()){
//      robot.wheels.Disable();
//      delay(1000);     
//      if (robot.fighter.ExtinguishFire()){
//        Serial.println("Extinguish fire in the main code is called");
//          //delay(1000);
//        robot.wheels.Attach();
//        robot.wheels.Strafe(true, 0);
//        delay(1000);
//      }
//}


//   Serial.print(LL.getRawReading());
//   Serial.print("   ");
//   Serial.print(LC.getRawReading());
//   Serial.print("   ");
//   Serial.print(RC.getRawReading());
//   Serial.print("   ");
//   Serial.println(RR.getRawReading());


 
//   Serial.print(LC.getDistance());
//   Serial.print("   ");
//   Serial.println(RC.getDistance());

    
//    Serial.println("Raw reading");
//    Serial.println(robot.lightInfo->PT_LL->getRawReading());
//    Serial.println(robot.lightInfo->PT_LC->getRawReading());
//    Serial.println(robot.lightInfo->PT_RC->getRawReading());
//    Serial.println(robot.lightInfo->PT_RR->getRawReading());
//
//    Serial.println("Moving average:");
//    Serial.println(robot.lightInfo->PT_LL->getAverageReading());
//    Serial.println(robot.lightInfo->PT_LC->getAverageReading());
//    Serial.println(robot.lightInfo->PT_RC->getAverageReading());
//    Serial.println(robot.lightInfo->PT_RR->getAverageReading());
    

//    robot.CL_Turn(-45);
//    robot.wheels.Disable();
//    //Serial.println(robot.gyro.GyroRead());
//    delay(3000);
//    robot.wheels.Attach();
//    robot.CL_Turn(-45);
//    robot.wheels.Disable();
//    delay(3000);
//    robot.wheels.Attach();
//    robot.CL_Turn(180);
//    robot.wheels.Disable();
//    delay(3000);
//    robot.wheels.Turn(false, 300);
//    robot.wheels.Disable();
//    delay(3000);

 // Serial.println("average reading called" + String(robot.RR_IR.getAverageReading()));
 // Serial.println("raw reading" + String(robot.RR_IR.getReading()));
  
//  Serial.print(robot.LF_IR.getAverageReading());
//  Serial.print("   ");
//  Serial.print(robot.RF_IR.getAverageReading());
//  Serial.print("   ");
//  Serial.print(robot.LR_IR.getAverageReading());
//  Serial.print("   ");
//  Serial.print(robot.RR_IR.getAverageReading());
//  Serial.print("   ");
//  Serial.println(robot.sonar.ReadUltraSonic());
//  Serial.println("   ");
//  delay(1000);
  


}
