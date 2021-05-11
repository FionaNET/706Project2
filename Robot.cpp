#include <Robot.h>
#define obsticalThresh 8

void Robot::Robot(){
    this->LF_IR = IR_Sensor(LONG,IR_LF);
    this->RF_IR = IR_Sensor(LONG,IR_RF);
    this->LR_IR = IR_Sensor(MID,IR_LR);
    this->RR_IR = IR_Sensor(MID,IR_RR);

    this->wheels = RobotBase(LFpin, RFpin, LRpin, RRpin);

    this->gyro = Gyroscope();
    this->sonar = Ultrasonic();
}

void Robot::rotate_while_scan(){
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

// void Robot::obstical_avoid(){
//     //Read all IR_Sensors and decide if any is too much
//     unsigned long startTime, strafeTime;

//     if(LF_IR.getReading() < obsticalThresh){
//         backPass = false;
//         //Strafe right until the path is clear in front of robot
//         while(LF_IR.getReading() < obsticalThresh){
//             startTime = millis();
//             wheels.Strafe(RIGHT, 0);
//         }
//         strafeTime = millis() - startTime;
//         //Keep going forward until the back has passed
//         while(!backPass){
//             wheels.Straight(400);
//             backPass = (LR_IR.getReading() < obsticalThresh);
//         }
//         //Go back left
//         wheels.Strafe(LEFT, strafeTime);

//     }else if (RF_IR.getReading() < obsticalThresh){
//         backPass = false;
//         //Strafe right until the path is clear in front of robot
//         while(RF_IR.getReading() < obsticalThresh){
//             startTime = millis();
//             wheels.Strafe(LEFT, 0);
//         }
//         strafeTime = millis() - startTime;
//         while(!backPass){
//             wheels.Straight(400);
//             backPass = (LR_IR.getReading() < obsticalThresh);
//         }
//         //Go back to correct path once object is passed
//         wheels.Strafe(RIGHT, strafeTime);
//     }

    //obstical avoidance with fuzzy logic
    void Robot::obstical_avoid(){
        if(LF_IR.isObject() && RF_IR.isObject() && sonar.isObject()){
            //All three sensors are reading objects so it is a wall
            wheels.CL_Turn(90);

        }else if(LF_IR.isObject() || RF_IR.isObject() || sonar.isObject()){
            //1 or more objects detected so it is a cyclindar
            
        }
    }

    void Robot::CL_Turn(int ref_angle){
    float kp_angle = 4;
    float ki_angle = 0.8;
    float cumulation;
    //speed value added to the motors 
    float rot_change, angle_error; 

    float turn_start = millis();
    float turn_time = 0;

    gyro.currentAngle = 0;      //reset the current angle in gyro

    angle_error = ref_angle - gyro.GyroRead();

    while (abs(angle_error) > angle_thres && turn_time <= 2000) {       //will exit while loop if the time in the while loop exceeds 2sec
    
    if(abs(angle_error) < 12.0){
      cumulation = cumulation + angle_error;
    }

    //noticed the robot had a problem where at the start of turning, it would think it'll need to turn CCW due to a negative angle error
    //since the robot is always turning CW, we absolute the rot_change for large angle_error 
    if(abs(angle_error) > 80){
      rot_change = abs(kp_angle * angle_error + ki_angle*cumulation);
    }else{
      rot_change = kp_angle * angle_error + ki_angle*cumulation;
    }

    rot_change = constrain(rot_change, -500, 500);
    
    Serial.print("The rot_change is: ");
    Serial.println(rot_change);
    
    //set the motors
    if(rot_change < 0){
      wheels.Turn(CCW, abs(rot_change));
    }else
    {
      wheels.Turn(CW, rot_change);
    }

    //read the gyro sensor and get current angle
    angle_error = ref_angle - gyro.GyroRead();
    Serial.print("The angle error is: ");
    Serial.println(angle_error);
    turn_time = millis() - turn_start;
  }
  }