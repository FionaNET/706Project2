#include <IR_Sensor.h>

//constructor
IR_Sensor::IR_Sensor(bool range, int pin){
    this->range = range;
    this->pin = pin;
}

//Get the reading
float IR_Sensor::getReading(){
    int voltage;
    float distance;

    voltage = analogRead(pin);

    if(range){      //true is mid range sensor
        //Return formula for calibrated ir mid range
    }else{          //false is long range sensor
        //Return formular for calibrated ir long range
    }
}

bool IR_Sensor::isObject(){
    return this->getReading() < this->objectThresh;
}