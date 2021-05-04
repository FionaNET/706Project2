#include <Phototransistor.h>

void Phototransistor::Init(uint8_t setSensorPin, int filterlenth){
    pinMode(PHOTOTRANSISTOR1, INPUT);

    this->currentVar = analogRead(PHOTOTRANSISTOR1);
    this->queue[fitlerlenth] = {};
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
