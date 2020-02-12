/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 11:10:55 
 * @Desc: main source file for the attitude estimation code 
 */

#include <Arduino.h>
#include "Config.h"

float acc[3] = {0.0};
float gyro[3] = {0.0};

unsigned long curTime = millis();
unsigned long preTime = 0;
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  LogInfo("attitude estimation program starts\n");
  IMU_Init();
  IMU_CalculateGyroBias();
}

void loop() {
  curTime = millis();
  // read imu data every IMU_SAMPLE_TIME ms
  if (curTime - preTime >= IMU_SAMPLE_TIME) {
    IMU_Read(acc, gyro);
    IMU_AccCalibrate(acc);
    IMU_GyroCalibrate(gyro);
    preTime = curTime;
  }
  
  if (curTime - lastLogTime >= 20) {
    IMU_PrintData(acc, gyro);
    // IMU_PrintAccPitchRoll(acc);
    lastLogTime = curTime;
  }
}