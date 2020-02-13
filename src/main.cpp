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

// array indices 0, 1, 2 corrsepond to x, y, z, respectively
float acc[3] = {0.0};
float gyro[3] = {0.0};
float accI[3] = {0.0}; // accel inertial frame

unsigned long curTime = millis();
unsigned long preTime = 0;
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  LogInfo("attitude estimation program starts\n");
  // MatrixTests();
  // AttitudeTests();
  IMU_Init();
  delay(1000); // small delay needed here to give gyro time
               // to warm up before sampling 
  AttitudeEulerInit(&angles);
  LogInfo("filling acc moving average buffer\n");
  for (int i = 0; i < 2*SAMPLES_ACC; i++) {
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
    IMU_GyroCalibrate(gyro);
    // perform the estimation algorithm of the DCM
    AttitudeClosedLoopIntegrationAcc(R, nR, gyro, acc, accI);
    // convert DCM into yaw, pitch, roll angles
    AttitudeDcmToEuler(nR, &angles);
    MatrixCopy(nR, R);

    preTime = curTime;
  }
  
  if (curTime - lastLogTime >= 50) {
    // IMU_PrintData(acc, gyro);
    // IMU_PrintGyroData(gyro);
    // IMU_PrintAccData(acc);
    // IMU_PrintAccPitchRoll(acc);
    // MatrixPrint(nR);
    AttitudePrintEuler(&angles);
    lastLogTime = curTime;
  }
}