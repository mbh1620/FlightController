#ifndef COMPLIMENTARY_FILTER_H
#define COMPLIMENTARY_FILTER_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

extern long lastInterval;
extern long lastTime;

extern float  accelX,             accelY,             accelZ,           //Units m/s/s   accelz should be 9.8 as its gravity
              gyroX,              gyroY,              gyroZ,            //units degrees per second
              gyroDriftX,         gyroDriftY,         gyroDriftZ,       //Units dps
              gyroRoll,           gyroPitch,          gyroYaw,          //Units degrees (expect major drift)
              gyroCorrectedRoll,  gyroCorrectedPitch, gyroCorrectedYaw, //Units degrees
              accRoll,            accPitch,           accYaw,           //Units degrees (expect minor drift)
              complementaryRoll,  complementaryPitch, complementaryYaw; //Units degrees (excellent roll, pitch and yaw minor drift

void calibrateIMU(int delayMillis, int calibrationMillis);

bool readIMU();

void doCalculations();

void printCalculations();

#endif
