/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 12:22:59 
 * @Desc: library to read and calibrate 
 * data from the LSM9DS1  IMU
 */

#ifndef IMU_H
#define IMU_H

#include "Config.h"

bool IMU_Init(void);
void IMU_Read(float acc[3], float gyro[3]);
void IMU_AccCalibrate(float acc[3]);
void IMU_GyroCalibrate(float gyro[3]);
void IMU_CalculateGyroBias(void);
void IMU_PrintData(float acc[3], float gyro[3]);
void IMU_PrintGyroData(float gyro[3]);
void IMU_PrintAccPitchRoll(float acc[3]);

#endif /* IMU_H */