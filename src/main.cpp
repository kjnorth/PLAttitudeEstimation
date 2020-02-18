/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 11:10:55 
 * @Desc: main source file for the attitude estimation code 
 */

#include <Arduino.h>
#include "Config.h"

// initial guess for R is the identity matrix
float R[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
};
float nR[3][3] = {0.0}; // this is the updated DCM accounting for gyro bias
// struct to hold yaw, pitch, and roll angles
struct Euler angles;
struct Euler anglesTemp;

// array indices 0, 1, 2 corrsepond to x, y, z, respectively
float acc[3] = {0.0};
float gyro[3] = {0.0}; // raw data
float gyroD[3] = {0.0}; // degrees
float gyroR[3] = {0.0}; // radians
float accI[3] = {0.0}; // accel inertial frame

unsigned long curTime = millis();
unsigned long preTime = 0;
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  LogInfo("attitude estimation program starts\n");
  IMU_Init();
  delay(1000); // small delay needed here to give gyro time
               // to warm up before sampling 
  AttitudeEulerInit(&angles);
  AttitudeEulerInit(&anglesTemp);
  LogInfo("filling acc moving average buffer\n");
  for (int i = 0; i < 2*SAMPLES_ACCEL; i++) {
    IMU_Read(acc, gyro);
    IMU_AccCalibrate(acc);
    delay(IMU_SAMPLE_TIME);
  }
  IMU_CalculateGyroBiasAndAccInertial(accI);
}

void loop() {
  curTime = millis();
  // read imu data every IMU_SAMPLE_TIME ms
  if (curTime - preTime >= IMU_SAMPLE_TIME) {
    IMU_Read(acc, gyro);
    IMU_AccCalibrate(acc);

    gyroD[0] = gyro[0];
    gyroR[0] = gyro[0];
    gyroD[1] = gyro[1];
    gyroR[1] = gyro[1];
    gyroD[2] = gyro[2];
    gyroR[2] = gyro[2];
    
    // for closed loop integration complimentary filter
    IMU_GyroCalibrateRPS(gyroR);
    // perform the estimation algorithm of the DCM
    AttitudeClosedLoopIntegrationAcc(R, nR, gyroR, acc, accI);
    // convert DCM into yaw, pitch, roll angles
    AttitudeDcmToEuler(nR, &angles);
    MatrixCopy(nR, R);

    // for much simpler complimentary filter
    IMU_GyroCalibrateDPS(gyroD);
    AttitudeComplimentaryFilter(gyroD, acc, &anglesTemp);

    preTime = curTime;
  }
  
  if (curTime - lastLogTime >= 100) {
    // for much smipler complimentary filter
    // LogInfo("simple pitch: ", anglesTemp.theta, 2);
    // LogInfo(", roll: ", anglesTemp.phi, 2);
    // for data logging
    LogInfo("", anglesTemp.theta, 2);
    LogInfo(", ", anglesTemp.phi, 2);
    
    // for closed loop integration complimentary filter
    AttitudePrintEuler(&angles);

    lastLogTime = curTime;
  }
}