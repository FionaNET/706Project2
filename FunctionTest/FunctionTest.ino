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
  //NEED TO ATTACH
  robot.wheels.Attach(); //Must call within setup or loop
  robot.gyro.GyroscopeCalibrate();
  //robot.gyro.currentAngle = 0;
  //delay(4000); UNCOMMENT LATER
  Serial.println("Start main loop");

}

int search = 0;
bool target_reached = false;
bool start = true;
float time_start = 0;
int fire = 0;                 //

void loop(){
  
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
//      if (fighter.ExtinguishFire()){
////        Serial.println("Extinguish fire in the main code is called");
//          //delay(1000);
//        robot.wheels.Attach();
//        robot.wheels.Strafe(true, 0);
//        delay(1000);
//      }
//      

  //blowing out 2 fires take highest priority
  if (fire == 2) {
    robot.wheels.Disable();

  //blowing out the fire is second priority because it might trigger obstacle avoidance
  } else if (target_reached) {
     robot.wheels.Disable();
     delay(1000); //give time for the motors to actually stop before turning on fan

     //extinguish the fire
     if (fighter.ExtinguishFire()) {
      delay(1000); //give time for the fan to fully turn off
      robot.wheels.Attach();
      //robot.wheels.Strafe(true, 0);
      //delay(1000);

      //reinitialise so we can extinguish the next fire
      fighter.Fire_extinguish = 0;

      //search again
      start = true;
      search = 0;

      //find the next target
      target_reached = false;
      
      //increase the number of fires we put out
      fire = fire + 1;
     }
     
  // ADD ANOTHER ELSE IF FOR OBSTACLE AVOIDANCE WHICH IS THIRD HIGHEST PRIORITY
  // Based on if any of the range sensors detect an object
  
  } else if (search != 1) {
    //rotate at the start or after 1 second of going straight
    if ((start == true) || (millis()-time_start) > 1000){
      search = robot.rotate_while_scan(true);
      time_start = millis(); 
      start = false;
    }
    //go straight for 1 sec
    robot.wheels.Straight(200);
  } else if (!target_reached) {
    target_reached = robot.go_target();
  } 
      
  //}
//   Serial.print(LL.getRawReading());
//   Serial.print("   ");
//   Serial.print(LC.getRawReading());
//   Serial.print("   ");
//   Serial.print(RC.getRawReading());
//   Serial.print("   ");
//   Serial.println(RR.getRawReading());


 
//   Serial.print(LC.getDistance());
//   //Serial.print(distance);
//   Serial.print("   ");
//   Serial.println(RC.getDistance());

   


    //robot.rotate_while_scan();
    //wheels.Turn(true, 5);

    
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
    //robot.obstical_avoid();
    

    //firefighting 
    //fighter.ExtinguishFire();

    //robot.obstacle_Avoid();
    //robot.wheels.Straight(200);
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
      //robot.wheels.Turn(false, 300);
     // robot.wheels.Disable();
      //delay(3000);
}
