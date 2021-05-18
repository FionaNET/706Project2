#ifndef Phototransistor_h
#define Phototransistor_h

#include "PinAllocation.h"
#include <Arduino.h>


class Phototransistor {
    public: 
        Phototransistor(uint8_t setSensorPin, int filterlenth);
        float getRawReading(); // get the brightness reading from sensor
        float getAverageReading();
        float getDistance();   //covert ADC value to distance (only for centre phototransistors)


    private:
        uint8_t setSensorPin;
        int filterlenth;


        float currentVar;
        float* queue; 
        int indx;
        float sum;
        float average;
       
};

#endif