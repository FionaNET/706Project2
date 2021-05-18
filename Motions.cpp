#include <Motions.h>
#include <IR_Sensor.h>
#include <RobotBase.h>
#include <Arduino.h>
#include <math.h>

Motions::Motions(){
    this->lightInfo = new LightDetect();
    this-> Robot = new RobotBase();

    backTracker = false;
}


void Motions::rotate_while_scan(){
    bool front = lightInfo->detect_front();
    float timeStart = millis();
    float timeout = 2000; // 2 sec for a full rotation, need calibration
    while (!front) {
        Robot->Turn(true, 10); // trun right 360 deg and scan
        front = lightInfo->detect_front();
        if (millis()-timeStart > timeout) {
            break;
        }
    }
    Robot->Stop();
    // alternative:
    // Robot.Disable();  

}

void Motions::obstical_avoid(){
    //Initialise the IR sensors
    IR_Sensor Left_FrontIR = IR_Sensor(LONG,IR_LF);
    IR_Sensor Left_RearIR = IR_Sensor(LONG,IR_RF);
    IR_Sensor Right_FrontIR = IR_Sensor(MID,IR_LR);
    IR_Sensor Right_rearIR = IR_Sensor(MID,IR_RR);


    //Read all IR_Sensors and decide if any is too much
    if(Left_FrontIR.getReading() < obsticalThresh){
        backTracker = true;
        
        while(backTracker){

        }
    }
}