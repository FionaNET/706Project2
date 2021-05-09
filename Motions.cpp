#include <Motions.h>

Motions::Motions(){
    this->lightInfo = LightDetect();
    this-> Robot = RobotBase();

}


void Moitions::rotate_while_scan(){
    bool front = lightInfo.detect_front();
    float timeStart = millis();
    float timeout = 2000; // 2 sec for a full rotation, need calibration
    while (!front) {
        Robot.Turn(true, 10); // trun right 360 deg and scan
        front = lightInfo.detect_front();
        if (millis()-timeStart > timeout) {
            break;
        }
    }
    Robot.stop();
    // alternative:
    // Robot.Disable();  

}