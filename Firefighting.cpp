#include <Firefighting.h>

Firefighting::Firefighting(int ThreshholdVal)
{
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

}

void Firefighting::FanOff(void)
{
    
}

bool Firefighting::ExtinguishFire(void) 
{
    
}