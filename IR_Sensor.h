#ifndef IR_Sensor_h
#define IR_Sensor_h

#include "PinAllocation.h"
#include <Arduino.h>
#define MID true
#define LONG false

class IR_Sensor{
    public:
    IR_Sensor();
    IR_Sensor(bool range, int pin);
    float getReading();
    bool isObject();

    private:
    bool range;
    int pin;
    int objectThresh;
    int offset;
    int wallThresh;
};

#endif