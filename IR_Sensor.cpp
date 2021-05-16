
#include "IR_Sensor.h"

//constructor
IR_Sensor::IR_Sensor(bool range, int pin){
    this->range = range;
    this->pin = pin;
}

//Get the reading
float IR_Sensor::getReading(){
    float voltage;
    float distance;

    voltage = analogRead(this->pin);

    if(this->pin == A5){                //IR Right Rear
        //Return formula for calibration
        distance = pow(voltage / (1.463*10^4), -(1 / 0.8801));

    }else if (this->pin == A4){         //IR Left Rear
        //Return formular for calibration
        distance = pow(voltage / (1.29*10^4), -(1 / 0.8457));

    } else if (this->pin == A6){        //IR Right Front
        distance = pow(voltage / (1.638*10^4), -(1 / 0.8053));

    } else {                           //IR Left Front (pin A7)
        distance = pow(voltage / (1.638*10^4), -(1 / 0.7698));
    }

    return distance;
}

bool IR_Sensor::isObject(){
    return this->getReading() < this->objectThresh;
}