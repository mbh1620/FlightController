#include <DFRobot_ICP10111.h>

#include <complimentary_filter.h>
#include <pid_loop.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

PID_LOOP X_PID(0.8, 0, 0);
PID_LOOP Y_PID(0.8, 0, 0);
PID_LOOP Z_PID(0.5, 0, 0);

PID_LOOP GPS_LAT_PID(0.3, 0, 0);
PID_LOOP GPS_LONG_PID(0.3, 0, 0);
PID_LOOP ALTITUDE_PID(0.6, 0, 0);

float motor1;
float motor2; 
float motor3;
float motor4;
float throttle = 40; //Throttle a percentage between 0-100 (will be controlled by PID once altitude measurement is done)

int MIN = 1000;
int MAX = 2000;

void setup() {
  Serial.begin(9600);

  Motor1.attach(2, MIN, MAX); //pin D2
  Motor2.attach(3, MIN, MAX); //pin D3
  Motor3.attach(5, MIN, MAX); //pin D5
  Motor4.attach(6, MIN, MAX); //pin D6

  Motor1.write(MIN);    //Starting speed 83 -> 90
  Motor2.write(MIN);
  Motor3.write(MIN);
  Motor4.write(MIN);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!"); 
  }

  //Initialise the Barometer here 
  //To estimate the height, a barometer, IMU and complementary filter is needed.
  //https://www.miro.ing.unitn.it/complementary-filtering-for-height-estimation-and-control-of-a-drone/
  //https://forum.arduino.cc/t/altitude-estimation-from-barometer-and-accelerometer-fusion-algoritm/296757/6
  
  calibrateIMU(250, 250);

  Serial.print("Motor1");
  Serial.print("Motor2");
  Serial.print("Motor3");
  Serial.println("Motor4");
 
}

void loop() {

  if(readIMU()){
    long currentTime = micros();
    lastInterval = currentTime - lastTime;
    lastTime = currentTime;

    doCalculations();
    }

   //Add function for receiving comms here 

   //---- pseudo code -----
   // 
   //  Take commands from comms and reset the desired points for all six PIDS, do this multiple times a second. The pids will then correct to minimise the error to the new set points.
   //  
   //  ALTITUDE = 
   //  LATITUDE =
   //  LONGITUDE = 
   //  
   //  ROLL = 
   //  PITCH =
   //  YAW = 

   //IF COMMUNICATIONS DROP THEN SET PRIMARY PIDS TO ZERO and SET SECONDARY PIDS TO CURRENT LOCATION

   //----------------------------------------
   //             Compute PIDS
   //----------------------------------------

   //Compute Linear (Secondary) PID Loops
   float lat_pid_output = GPS_LAT_PID.compute_loop(0, ACTUAL_GPS_LAT);
   float long_pid_output = GPS_LONG_PID.compute_loop(0, ACTUAL_GPS_LONG);
   float altitude_pid_output = GPS_ALTITUDE_PID.compute_loop(0, MEASURED_ALTITUDE);

   //Add code here to set the desired points of primary pids based on the secondary pids
   //Make sure that the Primary PIDS do not exceed a certain amount say 45 degrees
 
   //compute Rotational (Primary) PID Loops
   float x_pid_output = X_PID.compute_loop(0, complementaryRoll);
   float y_pid_output = Y_PID.compute_loop(0, complementaryPitch);
   float z_pid_output = Z_PID.compute_loop(0, complementaryRoll);
    
    //   X 4      X 1
    //    \      /                ^x
    //     \____/                 |
    //     |    |                 |--->y
    //     |____|                 z
    //    /      \                
    //   /        \               
    //  X 3        X 2
    
    motor1 = throttle-x_pid_output+y_pid_output+z_pid_output; 
    motor2 = throttle+x_pid_output+y_pid_output-z_pid_output; 
    motor3 = throttle+x_pid_output-y_pid_output+z_pid_output;
    motor4 = throttle-x_pid_output-y_pid_output-z_pid_output;
    
    motor1 = constrain(map(motor1, 0, 100, 1000, 2000), 1000, 2000);
    motor2 = constrain(map(motor2, 0, 100, 1000, 2000), 1000, 2000);
    motor3 = constrain(map(motor3, 0, 100, 1000, 2000), 1000, 2000);
    motor4 = constrain(map(motor4, 0, 100, 1000, 2000), 1000, 2000);

    Serial.print(motor1);
    Serial.print('\t');
    Serial.print(motor2);
    Serial.print('\t');
    Serial.print(motor3);
    Serial.print('\t');
    Serial.println(motor4);
    
    Motor1.write(motor1);   
    Motor2.write(motor2);
    Motor3.write(motor3);
    Motor4.write(motor4);

  }
