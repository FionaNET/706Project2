
#include "IR_Sensor.h"
#include <Arduino.h>
#include <math.h> 
//constructor

IR_Sensor::IR_Sensor(){

}

IR_Sensor::IR_Sensor(bool range, int pin){
    this->range = range;
    this->pin = pin;
    this->objectThresh = 140;
    this->offset = 10;
    this->wallThresh = 120;
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
        return (getReading() < this->objectThresh);
    }else{
        return (getReading() < this->wallThresh);
    }
    

}