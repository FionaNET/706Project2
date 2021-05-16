#include <Phototransistor.h>

Phototransistor::Phototransistor(uint8_t setSensorPin, int filterlenth){
    pinMode(setSensorPin, INPUT);

    this->currentVar = analogRead(setSensorPin);
    this->queue[filterlenth] = {};
    uint8_t indx = 0;
    float sum = 0;
    float average = 0;

}

float Phototransistor::getRawReading(){
    return this->currentVar;
}

float Phototransistor::getAverageReading(){
    this->sum = this->sum - this->queue[indx];
    this->queue[this->indx] = this->currentVar;
    this->sum = this->sum + this->currentVar;
    this->indx = (this->indx +1) % this->filterlenth;

    this->average = this->sum / this->filterlenth;

    return this->average;
}
