#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "RobotBase.h"
#include "Robot.h"


Firefighting fighter = Firefighting(2);
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
  

}

void loop(){

//robot.rotate_while_scan();
robot.go_target();
//   Serial.print(LL.getRawReading());
//   Serial.print("   ");
//   Serial.print(LC.getRawReading());
//   Serial.print("   ");
//   Serial.print(RC.getRawReading());
//   Serial.print("   ");
//   Serial.println(RR.getRawReading());

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

    robot.obstacle_Avoid();

    
    

}
