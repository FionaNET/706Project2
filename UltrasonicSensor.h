#include <PinAllocation.h>
#include <Arduino.h>

class UltrasonicSensor {
    public: 
        //constructor
        UltrasonicSensor(void);

        //functions
        float ReadUltraSonic(void);
};