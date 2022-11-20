//------------------------------------------------------------------------------------
//                         Quadcopter Flight Controller Code V1.0
//------------------------------------------------------------------------------------
/*
 * Quadcopter Flight Controller Software uses a Arduino Nicla Sense ME. Nicla Sense is 
 * connected to a Raspberry Pi Zero 2W using I2C. 
 */
// Matthew Haywood

#include "Arduino_BHY2.h"
#include <Wire.h>
#include <Servo.h>
#include <pid_loop.h>
#include <moving_average_filter.h>

//Initialise Motors

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

float motor1;
float motor2;
float motor3;
float motor4;

int MIN = 1000;
int MAX = 2000;

//Inititialise Sensors

Sensor device_orientation(SENSOR_ID_DEVICE_ORI);
SensorOrientation orientation(SENSOR_ID_ORI);
Sensor pressure(SENSOR_ID_BARO);

//Initialise Axis and altitude PIDS

PID_LOOP X_PID(0.6, 2, 2);
PID_LOOP Y_PID(0.8, 0, 0);
PID_LOOP Z_PID(0.2, 0, 0);
PID_LOOP ALT_PID(2, 0, 0);

Moving_Average_Filter filter(4);

float measured_altitude;
float starting_pressure;
float altitude_difference;

    void setup(){

    //Setup I2C connection
    Wire.begin(8);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
    
    //Begin various sensors
    BHY2.begin();
    pressure.begin();
    orientation.begin();
    delay(1000);
    BHY2.update();

    //Setup and attach Motors

    Motor1.attach(9, MIN, MAX);
    Motor2.attach(7, MIN, MAX);
    Motor3.attach(8, MIN, MAX);
    Motor4.attach(6, MIN, MAX);

    Motor1.write(MAX);
    Motor2.write(MAX);
    Motor3.write(MAX);
    Motor4.write(MAX);
    delay(4000);

    Motor1.write(MIN);
    Motor2.write(MIN);
    Motor3.write(MIN);
    Motor4.write(MIN);
    delay(4000);
    
    Motor1.write(MIN);
    Motor2.write(MIN);
    Motor3.write(MIN);
    Motor4.write(MIN);
    
    starting_pressure = pressure.value();
    
  }
  
  void receiveEvent(int howMany){
    while(1 < Wire.available()){
      char x = Wire.read();
      Serial.print(x);
    }
    char x = Wire.read();
    Serial.println(x);
  }

  void loop(){
    BHY2.update();

    measured_altitude = (44330 * (1-pow((pressure.value()/1013.13),(1.0/5.255)))); //<------ This may need to be changed to relative altitude. Need to add a moving average filter similar to joop brokking
    
    altitude_difference = measured_altitude - (44330 * (1-pow((starting_pressure/1013.13),(1.0/5.255))));
    float filtered = filter.Moving_Average_Compute(altitude_difference);
    float altitude_pid_output = ALT_PID.compute_loop(10,  altitude_difference);  //<-- Desired setpoint will be desired altitude change from where the drone started.
    float x_pid_output = X_PID.compute_loop(0, orientation.roll());  //Will need to change to Quaternions as does not seem to be working correctly
    float y_pid_output = Y_PID.compute_loop(0, orientation.pitch()); 
    float z_pid_output = Z_PID.compute_heading_loop(25, orientation.heading()); //Desired set point is absolute heading degrees

    //   X 4      X 1
    //    \      /                ^x
    //     \____/                 |
    //     |    |                 |--->y
    //     |____|                 z
    //    /      \                
    //   /        \               
    //  X 3        X 2

    motor1 = altitude_pid_output-x_pid_output-y_pid_output+z_pid_output;
    motor2 = altitude_pid_output+x_pid_output-y_pid_output-z_pid_output;
    motor3 = altitude_pid_output+x_pid_output+y_pid_output+z_pid_output;
    motor4 = altitude_pid_output-x_pid_output+y_pid_output-z_pid_output;

    motor1 = constrain(map(motor1, 0, 100, 1000, 2000), 1000, 2000);
    motor2 = constrain(map(motor2, 0, 100, 1000, 2000), 1000, 2000);
    motor3 = constrain(map(motor3, 0, 100, 1000, 2000), 1000, 2000);
    motor4 = constrain(map(motor4, 0, 100, 1000, 2000), 1000, 2000);

    Serial.println(orientation.roll());
//    Serial.print(orientation.pitch());
//    Serial.print('\t');
//    Serial.println(orientation.heading());
//    Serial.println(altitude_difference);
//    Serial.print('\t');
//    Serial.println(motor4);
//    Serial.print('\t');
//    Serial.println(filtered);
//    Serial.print('\t');
//    Serial.println(altitude_difference);
//    Serial.print(motor1);
//    Serial.print('\t');
//    Serial.print(motor2);
//    Serial.print('\t');
//    Serial.print(motor3);
//    Serial.print('\t');
//    Serial.println(motor4);
    
    
    Motor1.write(motor1);
    Motor2.write(motor2);
    Motor3.write(motor3);
    Motor4.write(motor4);
    
    delay(10);
  }
