#include <Robot.h>
#define obsticalThresh 8

void Robot::Robot(){
    this->LF_IR = IR_Sensor(LONG,IR_LF);
    this->RF_IR = IR_Sensor(LONG,IR_RF);
    this->LR_IR = IR_Sensor(MID,IR_LR);
    this->RR_IR = IR_Sensor(MID,IR_RR);

    this->lightInfo = new LightDetect();

    this->wheels = RobotBase(LFpin, RFpin, LRpin, RRpin);

    this->gyro = Gyroscope();
    this->sonar = Ultrasonic();

    this->speed_step = 1; // set the speed stepsize to be 1
}

void Robot::rotate_while_scan(){
    bool front = lightInfo.detect_front();
    float timeStart = millis();
    float timeout = 2000; // 2 sec for a full rotation, need calibration
    while (!front) {
        this->wheels.Turn(true, 10); // trun right 360 deg and scan
        front = lightInfo.detect_front();
        if (millis()-timeStart > timeout) {
            break;
        }
    }
    this->wheels.Stop();
    // alternative:
    // this->wheels.Disable();  

}

//void Robot::rotate_angle(float ref_angle){
//
//   float rotate_time = this->alpha * ref_angle / this->speed;
//
//   float time_start = millis();

 //   while (millis() - time_start < rotate_time){
 //       this->wheels->Turn(direction, speed);
 //   }
        
    
//}

void Robot::obstical_avoid(){
    //Read all IR_Sensors and decide if any is too much
    unsigned long startTime, strafeTime;

    if(LF_IR.getReading() < obsticalThresh){
        backPass = false;
        //Strafe right until the path is clear in front of robot
        while(LF_IR.getReading() < obsticalThresh){
            startTime = millis();
            wheels.Strafe(RIGHT, 0);
        }
        strafeTime = millis() - startTime;
        //Keep going forward until the back has passed
        while(!backPass){
            wheels.Straight(400);
            backPass = (LR_IR.getReading() < obsticalThresh);
        }
        //Go back left
        wheels.Strafe(LEFT, strafeTime);

    }else if (RF_IR.getReading() < obsticalThresh){
        backPass = false;
        //Strafe right until the path is clear in front of robot
        while(RF_IR.getReading() < obsticalThresh){
            startTime = millis();
            wheels.Strafe(LEFT, 0);
        }
        strafeTime = millis() - startTime;
        while(!backPass){
            wheels.Straight(400);
            backPass = (LR_IR.getReading() < obsticalThresh);
        }
        //Go back to correct path once object is passed
        wheels.Strafe(RIGHT, strafeTime);
    }


    void Robot::go_target(){
        int tar_arrive = 0;
        

        while (tar_arrive != 1) {
            bool direction; 

            this->obstical_avoid();

            float fuzzy_var = this->lightInfo->detect_dir();
            if (fuzzy_var <0){
                direction = false;
            }    


            float speed = fuzzy_var * this->speed_step;
            this->turn(direction,speed);
            delay(10); // allow rotation to happen
            this->wheels->forward();
            
            // check if meet target
            if ((this->sonar.ReadUltraSonic() < this->thr_sonar) && this->lightInfo->detect_front()){
                tar_arrive = 1;
            } 
        }

        // arrived at the target
         this->wheels.Stop();
        // alternative:
        // this->wheels.Disable(); 
    }