#ifndef Phototransistor_h
#define Phototransistor_h

#include "PinAllocation.h"
#include <Arduino.h>


class Phototransistor {
    public: 
        Phototransistor(int setSensorPin);
        float getRawReading(); // get the brightness reading from sensor
        int getAverageReading();
        float getDistance();   //covert ADC value to distance (only for centre phototransistors)


    private:
        uint8_t setSensorPin;
        float currentVar;
        float queue[FILTERLENGTH_P] = {0}; 
        int indx;
        float sum;
        float average;
       
};

#endif