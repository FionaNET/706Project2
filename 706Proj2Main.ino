#include <DetermineDirection.h>
#include <Motions.h>
#include <Firefighting.h>
#include <PinAllocation.h>
#include <Gyroscope.h>

DetermineDirection FireDetect = DetermineDirection();
Motions Robot = Motions();
Firefighting Firefighter = Firefighting();

int FireNum = 2; 
float ref_angle = 0;
void setup() {
  // put your setup code here, to run once:
  
  //Gyroscope calibration called via gyroscopecalibration();
  Gyroscope Gyro = new Gyroscope();
  Gyro.GyroscopeCalibrate();
}

void loop() {
    if (FireNum > 0){
       FireDetect.scan_while_rotate();
       if (FireDetect.exist){
          // ref_angle = FireDetect.get_target_dir();
          // Robot.turn(ref_angle);
          Robot.go_target();
          Firefighter.fight();
          if Firefighter.done {
            Robot.Stop;
            FireNum = FireNum - 1;
          }
       } 
       else {
        // go randomly? or some other position
       }
    }
}
