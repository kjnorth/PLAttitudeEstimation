/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 10:46:53 
 * @Desc: includes all files, holds pin definitions,
 * and holds #defines for the project 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "..\lib\Attitude\AttitudeTest.h"
#include "..\lib\Adafruit_LSM9DS1\Adafruit_LSM9DS1.h"
#include "..\lib\DataLog\DataLog.h"
#include "IMU.h"
#include "..\lib\Matrix\MatrixTest.h"

// IMU moving average
#define SHIFT_ACC 			                    3
#define SAMPLES_ACC 		                    (1 << SHIFT_ACC)
#define IMU_SAMPLE_TIME                         20 // ms
#define GYRO_SCALE_FACTOR                       0.00875 // 8.75 mdps/LSB
#define NUM_SAMPLES_NORMALIZE_GYRO_AVERAGE      100
#define NUM_GYRO_BIAS_SAMPLES                   100.0

#endif /* CONFIG_H */