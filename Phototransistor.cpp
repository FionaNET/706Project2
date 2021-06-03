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
    this->indx1 = 0; 
    this->sum1 = 0;
    this->ave_pre = 0;
    this->average1 = 0;
    this->prev2 = 0;
    this->prev1 = 0;


}

float Phototransistor::getRawReading(){
    // this->prev2 = this->prev1;
    // this->prev1 = this->currentVar;
    this->currentVar = analogRead(this->setSensorPin);

    // if(this->currentVar> 940)
    // {
    //    this->currentVar = (this->prev2 + this->prev1)/2;
    // }

//     this->queue1[this->indx1] = this->currentVar;
//     this->sum1 = (this->sum1 + this->currentVar);
//     // Serial.println("Current sum of middle two phtotransistor:");
//     // Serial.println(sum);
//     //Serial.println("Current index:");
//     //Serial.println(indx);
//     this->indx1 = ((this->indx1 +1) % FILTERLENGTH_P1);
//     // Serial.println("Current index after divisinon by filterlengths:");
//     //Serial.println(indx);
//    this->average1 = (this->sum1 / FILTERLENGTH_P1);
//     // Serial.println("Returned average:");
//     // Serial.println(average);
//     this->ave_pre = this->average1;

    return this->currentVar;
}

int Phototransistor::getAverageReading(){

    this->sum = this->sum - this->queue[indx];

    this->queue[this->indx] = this->currentVar;
    this->sum = (this->sum + this->currentVar);
    // Serial.println("Current sum of middle two phtotransistor:");
    // Serial.println(sum);
    //Serial.println("Current index:");
    //Serial.println(indx);
    this->indx = ((this->indx +1) % FILTERLENGTH_P);
    // Serial.println("Current index after divisinon by filterlengths:");
    //Serial.println(indx);
    this->average = (this->sum / FILTERLENGTH_P);
    // Serial.println("Returned average:");

    return this->average;
}

float Phototransistor::getDistance(){
    float distance;
    this->getRawReading();

    if (this->getRawReading() < 10){
        return 500;
    }

    if (this->setSensorPin == A9) {       //Centre left phototransistor
        distance = pow((this->currentVar/(1.429*pow(10,6))), -(1 / 1.736));
    } else {                              //Centre right phototransistor
        distance = pow((this->currentVar/(9.441*pow(10,5))), -(1 / 1.663)) ;
    }

    return distance;

}
