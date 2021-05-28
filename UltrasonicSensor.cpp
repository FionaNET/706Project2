#include <UltrasonicSensor.h>
#include <Arduino.h>
#include <math.h>
#define MAX_DIST 4000


UltrasonicSensor::UltrasonicSensor(void)
{
    this->objectThresh = 220;// need to adjust
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    
}

bool UltrasonicSensor::isObject(){
  return this->ReadUltraSonic() < this->objectThresh;
}

//OUTPUTS mm
float UltrasonicSensor::ReadUltraSonic(void)
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float mm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
//      SerialCom->println("HC-SR04: NOT found");
      return MAX_DIST;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
 //     SerialCom->println("HC-SR04: Out of range");
      return MAX_DIST;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  mm = (pulse_width / 58.0)*10;
  // Print out results
  if ( pulse_width > MAX_DIST ) {
//    SerialCom->println("HC-SR04: Out of range");
    return MAX_DIST;
  } else {
//    SerialCom->print("HC-SR04:");
//    SerialCom->print(cm);
//    SerialCom->println("cm");
    return mm;
  }
}