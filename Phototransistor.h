#include <PinAllocation.h>
#include <Arduino.h>


class Phototransistor {
    public: 
        Phototransistor(uint8_t setSensorPin, int filterlenth);
        float getRawReading(); // get the brightness reading from sensor
        float getAverageReading();


    private:
        uint8_t setSensorPin;
        int filterlenth;


        float currentVar;
        float* queue; 
        int indx;
        float sum;
        float average;
       
}