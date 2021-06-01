#include <Firefighting.h>
#include <Arduino.h>
#include <math.h>
#include <PinAllocation.h>

Firefighting::Firefighting(int ThreshholdVal)
{
    pinMode(FAN_PIN, OUTPUT); //set as output
    digitalWrite(FAN_PIN, LOW); // set fan to low by default
    this->Fire_extinguish = 0;
    this->LightDetector = new LightDetect(); 
    this->fireThreshhold = ThreshholdVal;
}

Firefighting::~Firefighting(void)
{
    delete this->LightDetector;
}

void Firefighting::FanOn(void)
{
    digitalWrite(FAN_PIN, HIGH); // set fan to low by default
}

void Firefighting::FanOff(void)
{
    Serial.println("Calling Fan Off in Firefighting cpp");
    digitalWrite(FAN_PIN, LOW); // set fan to low by default
}

bool Firefighting::ExtinguishFire(void) 
{
    this->FanOn(); //turn fan on
    float start_time = millis();
    while (this->Fire_extinguish == 0){
        if ((this->LightDetector->getPTAvg() < this->fireThreshhold) ||((millis() - start_time) > 5000) ){
            this->Fire_extinguish = 1;
        }
    }
    this->FanOff(); //turn fan off
    //delay(2500);
    return true;
}