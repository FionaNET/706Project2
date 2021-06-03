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

    this->fanPin = SERVO_PIN;
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

bool Firefighting::ExtinguishFire(double angle) 
{
  this->Fire_extinguish = 0;
  this->FanOn(); //turn fan on
  while (this->Fire_extinguish == 0){
    // this->servoCall(angle + 20);
    // delay(50);
    // this->servoCall(angle - 20);
    // delay(50);
    //if ((this->LightDetector->getPTAvg() < this->fireThreshhold) ||((millis() - start_time) > 5000) ){
    if ((this->LightDetector->getPTAvg() < this->fireThreshhold) ){
      this->servoReset();
      this->Fire_extinguish = 1;
    }
  }
  this->FanOff(); //turn fan off
  //delay(2500);
  return true;
}

void Firefighting::FanServoDisable() { 
  fanServo.detach();
  pinMode(this->fanPin, INPUT);
}

void Firefighting::FanServoAttach() { 
  fanServo.attach(this->fanPin);
  //pinMode(this->fanPin, OUTPUT);
}

void Firefighting::servoLeft(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MAX);
  this->fanServo.write(70);
  
}

void Firefighting::servoRight(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MIN);
  this->fanServo.write(50);
}

void Firefighting::servoCall(int angle){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MIN);
  this->fanServo.write(angle);
}

void Firefighting::servoReset(){
  //this->wheels->fanServo.writeMicroseconds(SERVO_MIDDLE);
  this->fanServo.write(60);
}

bool  Firefighting::servoRotate(){
  float LLAve = this->lights->PT_LL->getRawReading();
  float LCAve = this->lights->PT_LC->getRawReading();
  float RCAve = this->lights->PT_RC->getRawReading();
  float RRAve = this->lights->PT_RR->getRawReading();
  float angle = SERVO_MIDDLE;

  float centreError = RCAve - LCAve;
  //continue to rotate the servo until LC and RC have a high value
  while ((abs(centreError) > 200) || (LCAve < SERVO_TARGET_BRIGHTNESS) || (RCAve < SERVO_TARGET_BRIGHTNESS)) {
    // Serial.println("Right photo: " + String(RRAve) + " Left photo: " +  String(LLAve));
    // Serial.println("Right Centre photo: " + String(RCAve) + " Left Centre photo: " +  String(LCAve));
    // Serial.println("Centre error: " + String(centreError));
    // Rotate to the highest light value
    if (RRAve>LLAve || centreError > 0){
      angle = angle - 10; // turn right
      // Serial.println("Turn right, decrease angle");
    } else{
      angle = angle + 10; // turn left
      // Serial.println("Turn left, increase angle");
    }

    constrain(angle, SERVO_MIN , SERVO_MAX);
    // Serial.println("Angle: " + String(angle));
    this->fanServo.write(angle);

    LLAve = this->lights->PT_LL->getRawReading();
    LCAve = this->lights->PT_LC->getRawReading();
    RCAve = this->lights->PT_RC->getRawReading();
    RRAve = this->lights->PT_RR->getRawReading();
    centreError = RCAve - LCAve;

    delay(50);
  }

  return this->ExtinguishFire(angle); 
  // Serial.println("Light reached");

}