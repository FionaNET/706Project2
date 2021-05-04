
 float Gyroscope::GyroscopeCalibrate(){
// find the voltage value when gyro is zero
  int i;
  float sum = 0;
  float sensorValue = 0;
  pinMode(GYRO_PIN), INPUT);

  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");

  // read 100 values of voltage when gyro is at still to calculate the zero-drift
  for (i = 0; i < 100; i++)
  {
    sensorValue = analogRead(GYRO_PIN);
    sum += sensorValue;
    delay(5);
  }
  
  // average the sum to find the zero drift
  return sum / 100; 
 }
