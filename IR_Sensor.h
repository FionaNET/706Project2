#include <PinAllocation.h>
#include <Arduino.h>
#define MID true
#define LONG false

class IR_Sensor{
    public:
    IR_Sensor(bool range, int pin);
    float getReading();
    bool isObject();

    private:
    bool range;
    int pin;
    int objectThresh;
};