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
    int getAverageReading();

    private:
    bool range;
    int pin;
    int objectThresh;
    int offset;
    int wallThresh;

    // moving average varibles
    float queue[FILTERLENGTH_IR] = {0}; 
    int indx;
    float sum;
    float average;
};

#endif