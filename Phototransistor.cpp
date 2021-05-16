#include <Phototransistor.h>

Phototransistor::Phototransistor(uint8_t setSensorPin, int filterlenth){
    pinMode(setSensorPin, INPUT);
    this->currentVar = analogRead(setSensorPin);

    this->queue[filterlenth] = {};
    this->indx = 0;
    this->sum = 0;
    this->average = 0;

}

float Phototransistor::getRawReading(){
    this->currentVar = analogRead(setSensorPin);
    return this->currentVar;
}

float Phototransistor::getAverageReading(){

    this->sum = this->sum - this->queue[indx];
    this.getRawReading();
    this->queue[this->indx] = this->currentVar;
    this->sum = this->sum + this->currentVar;
    this->indx = (this->indx +1) % this->filterlenth;

    this->average = this->sum / this->filterlenth;

    return this->average;
}

float Phototransistor::getDistance(){
    float distance;
    this.getRawReading();

    if (this->setSensorPin == A9) {       //Centre left phototransistor
        distance = power( this->currentVar / (1.429*10^6), -(1 / 1.736));
    } else {                              //Centre right phototransistor
        distance = power( this->currentVar / (9.441*10^5), -(1 / 1.663));
    }

}
