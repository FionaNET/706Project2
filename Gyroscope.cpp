#include <Gyroscope.h>

Gyroscope::Gyroscope(void)
[
    pinMode(GYRO_PIN), INPUT);
    this.GyroZeroVoltage = 0;          //gyro zero drift voltage
    this.T = 100;                        // T is the time of one loop
    this.sensorValue = 0;                // read out value of sensor
    this.gyroSupplyVoltage = 5;        // supply voltage for gyro
    this.gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
    this.rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
    this.gyroRate = 0;                 // read out value of sensor in voltage
    this.currentAngle = 0;             // current angle calculated by angular velocity integral on
    this.serialRead = 0;                // for serial print control
]

 float Gyroscope::GyroscopeCalibrate(void)
 {
// find the voltage value when gyro is zero
  float sum = 0;
  
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");

  // read 100 values of voltage when gyro is at still to calculate the zero-drift
  for (int i = 0; i < 100; i++)
  {
    this.sensorValue = this.GyroRawRead();
    sum += this.sensorValue;
    delay(5);
  }
  this.GyroZeroVoltage = sum / 100; 
  // average the sum to find the zero drift
  return sum / 100; 
 }

float Gyroscope::GyroRead(void)
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(this.GyroRawRead());

  // convert the 0-1023 signal to 0-5v
  this.gyroRate = (this.GyroRawRead() * this.gyroSupplyVoltage) / 1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  this.gyroRate -= (this.GyroZeroVoltage / 1023 * 5);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = this.gyroRate / this.gyroSensitivity;

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= this.rotationThreshold || angularVelocity <= -1*(this.rotationThreshold)) {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / this.T);
    this.currentAngle += angleChange;
  }

  if (this.currentAngle < 0) {
    this.currentAngle += 360;
  } else if (this.currentAngle > 359) {
    this.currentAngle -= 360;
  }
  delay(this.T);
  return this.currentAngle;
}

float Gyroscope::GyroRawRead(void)
{
  return analogRead(GYRO_PIN);
}