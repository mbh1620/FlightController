#include <Wire.h>
#include <complimentary_filter.h>
//Setup all the variables

void setup() {
  Serial.begin(1000000);
  pinMode(LED_BUILTIN, OUTPUT);

  while(!Serial);

  if (!IMU.begin()){
    Serial.println("Failed to initialize IMU!");
    while(1);
  }

  calibrateIMU(250,250);
  lastTime = micros();

}


void loop() {
  // put your main code here, to run repeatedly:

  if(readIMU()){
    long currentTime = micros();
    lastInterval = currentTime - lastTime;
    lastTime = currentTime;

    doCalculations();
    printCalculations();
    
  }
  
}
