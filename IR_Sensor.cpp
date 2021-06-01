
#include "IR_Sensor.h"
#include <Arduino.h>
#include <math.h> 
//constructor

IR_Sensor::IR_Sensor(){

}

IR_Sensor::IR_Sensor(bool range, int pin){
    this->range = range;
    this->pin = pin;
    this->objectThresh = FrontObject;
    this->offset = 10;
    this->wallThresh = BackObject;

    this->indx = 0; 
    this->sum = 0;
    this->average = 0;
}

//Get the reading in mm
float IR_Sensor::getReading(){
    float voltage;
    float distance;

    voltage = analogRead(this->pin);

    if(this->pin == IR_RR){                //IR Right Rear
        //Return formula for calibration
        distance = pow((voltage / (1.463*pow(10,4))), (-(1 / 0.8801)));

    }else if (this->pin == IR_LR){         //IR Left Rear 
        //Return formular for calibration
        distance = pow((voltage / (1.29*pow(10,4))), (-(1 / 0.8457)));

    } else if (this->pin == IR_RF){        //IR Right Front
        distance = pow((voltage / (1.638*pow(10,4))), (-(1 / 0.8053)));

    } else {                           //IR Left Front (pin A7)
        distance = pow((voltage / (1.638*pow(10,4))), (-(1 / 0.7698)));
    }

    return distance;
}

bool IR_Sensor::isObject(){
    if(range == LONG){
        return (getAverageReading() < this->objectThresh);
    }else{
        return (getAverageReading() < this->wallThresh);
    }
}

int IR_Sensor::getAverageReading(){

    this->sum = this->sum - this->queue[this->indx];
    float currentVar = this->getReading();
    this->queue[this->indx] = currentVar;
    this->sum = (this->sum + currentVar);
    // Serial.println("Current sum of middle two phtotransistor:");
    // Serial.println(sum);
    //Serial.println("Current index:");
    //Serial.println(indx);
    this->indx = ((this->indx +1) % FILTERLENGTH_IR);
    // Serial.println("Current index after divisinon by filterlengths:");
    //Serial.println(indx);
    this->average = (this->sum / FILTERLENGTH_IR);
    // Serial.println("Returned average:");
    // Serial.println(average);
    return this->average;
}