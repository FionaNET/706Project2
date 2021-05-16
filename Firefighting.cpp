#include <Firefighting.h>

Firefighting::Firefighting(int ThreshholdVal)
{
    pinMode(FAN_PIN, OUTPUT); //set as output
    digitalWrite(FAN_PIN, LOW); // set fan to low by default
    this.Fire_extinguish = 0;
    this->LightDetector = new LightDetect(5,10,15); //example threshhold values
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
    digitalWrite(FAN_PIN, LOW); // set fan to low by default
}

bool Firefighting::ExtinguishFire(void) 
{
    this->FanOn(); //turn fan on
    while (this->Fire_extinguish == 0)
    {
        if (this.LightDetector->getPTAvg() > this->fireThreshhold)
        {
            this->Fire_extinguish = 1;
        }
    }
    this->FanOff(); //turn fan off
    return true;
}