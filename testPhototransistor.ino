#include <Phototransistor.h>
#include <PinAllocation.h>

Phototransistor PT = Phototransistor(PHOTOTRANSISTOR1,5);
float raw = 0;
float average = 0;

void setup(){

}

void loop(){
    raw = PT.getRawReading();
    avearge = PT.getAverageReading();
    Serial.println("Raw:");
    Serial.print(raw);
    Serial.println("Average");
    Serial.print(average);
}