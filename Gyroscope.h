#include <Arduino.h>
#include <Servo.h>
#include <PinAllocation.h>

class Gyroscope {
  public:
    // Constructor
    Gyroscope(void);

    //variables
    float GyroZeroVoltage;        //gyro zero drift voltage
    int T;                        // T is the time of one loop
    int sensorValue;                // read out value of sensor
    float gyroSupplyVoltage;        // supply voltage for gyro
    float gyroSensitivity;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
    float rotationThreshold;      // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
    float gyroRate;                 // read out value of sensor in voltage
    float currentAngle;             // current angle calculated by angular velocity integral on
    byte serialRead;                // for serial print control
    
    //functions
    // Calibrate the gyroscope which returns the zero voltage value
   float GyroscopeCalibrate(void); 
   float GyroRead(void);
   float GyroRawRead(void);
};