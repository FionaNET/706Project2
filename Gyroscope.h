#include <Arduino.h>
#include <Servo.h>
#include <PinAllocation.h>

class Gyroscope {
  public:
    // Constructor
    Gyro(void);

    // Methods that control robot movement
   float GyroscopeCalibrate(); 
};