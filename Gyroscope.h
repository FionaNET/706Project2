#include <Arduino.h>
#include <Servo.h>
#include <PinAllocation.h>

class Gyroscope {
  public:
    // Constructor
    Gyroscope(void);

    //variables
    float GyroZeroVoltage;
    //functions
    // Calibrate the gyroscope which returns the zero voltage value
   float GyroscopeCalibrate(void); 
};