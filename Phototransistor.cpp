#include <Phototransistor.h>
#include <Arduino.h>
#include <math.h>
Phototransistor::Phototransistor(int _setSensorPin){
    //pinMode(setSensorPin, INPUT);
    this->setSensorPin = _setSensorPin;
    this->currentVar = analogRead(this->setSensorPin);
    this->indx = 0;
    this->sum = 0;
    this->average = 0;

}

float Phototransistor::getRawReading(){
    this->currentVar = analogRead(this->setSensorPin);
    return this->currentVar;
}

int Phototransistor::getAverageReading(){

    this->sum = this->sum - this->queue[indx];
    this->getRawReading();
    this->queue[this->indx] = this->currentVar;
    this->sum = (this->sum + this->currentVar);
    Serial.println("Current sum of middle two phtotransistor:");
    Serial.println(sum);
    //Serial.println("Current index:");
    //Serial.println(indx);
    this->indx = ((this->indx +1) % FILTERLENGTH_P);
    Serial.println("Current index after divisinon by filterlengths:");
    //Serial.println(indx);
    this->average = (this->sum / FILTERLENGTH_P);
    Serial.println("Returned average:");
    Serial.println(average);
    return this->average;
}

float Phototransistor::getDistance(){
    float distance;
    this->getRawReading();

    if (this->setSensorPin == A9) {       //Centre left phototransistor
        distance = pow( this->currentVar / 1.429*pow(10,6), -(1 / 1.736)) ;
    } else {                              //Centre right phototransistor
        distance = pow( this->currentVar / 9.441*pow(10,5), -(1 / 1.663)) ;
    }

}
