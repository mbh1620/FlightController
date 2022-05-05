#include "complimentary_filter.h"

long lastInterval = 0;
long lastTime = 0;

float  accelX,             accelY,             accelZ,           //Units m/s/s   accelz should be 9.8 as its gravity
       gyroX,              gyroY,              gyroZ,            //units degrees per second
       gyroDriftX,         gyroDriftY,         gyroDriftZ,       //Units dps
       gyroRoll,           gyroPitch,          gyroYaw,          //Units degrees (expect major drift)
       gyroCorrectedRoll,  gyroCorrectedPitch, gyroCorrectedYaw, //Units degrees
       accRoll,            accPitch,           accYaw,           //Units degrees (expect minor drift)
       complementaryRoll,  complementaryPitch, complementaryYaw = 0; //Units degrees (excellent roll, pitch and yaw minor drift)


void calibrateIMU(int delayMillis, int calibrationMillis) {
  int calibrationCount = 0;
  delay(delayMillis);

  float sumX, sumY, sumZ;

  int startTime = millis();
  while (millis() < startTime + calibrationMillis){
    if(readIMU()) {
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if( calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;
}

bool readIMU() {
  if( IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}

void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX/lastFrequency);
  gyroPitch = gyroPitch + (gyroY/lastFrequency);
  gyroYaw = gyroYaw + (gyroZ/lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
    
}

void printCalculations() {
//  Serial.print(gyroRoll);
//  Serial.print(',');
//  Serial.print(gyroPitch);
//  Serial.print(',');
//  Serial.print(gyroYaw);
//  Serial.print(',');
//  Serial.print(gyroCorrectedRoll);
//  Serial.print(',');
//  Serial.print(gyroCorrectedPitch);
//  Serial.print(',');
//  Serial.print(gyroCorrectedYaw);
//  Serial.print(',');
//  Serial.print(accRoll);
//  Serial.print(',');
//  Serial.print(accPitch);
//  Serial.print(',');
//  Serial.print(accYaw);
//  Serial.print(',');
  Serial.print(complementaryRoll);
  Serial.print(',');
  Serial.print(complementaryPitch);
  Serial.print(',');
//  Serial.print(complementaryYaw);
  Serial.println("");
}
