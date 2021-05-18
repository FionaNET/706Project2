#include "Firefighting.h"
#include "PinAllocation.h"
#include "LightDetect.h"
#include "Robot.h"

Firefighting fighter = Firefighting(2);
LightDetect lightInfo = LightDetect();
Robot robot = Robot();

void setup(){

}

void loop(){

    robot.rotate_while_scan();
//    robot.obstical_avoid();
//    robot.go_target();

    // firefighting 
//    fighter.ExtinguishFire();


}
