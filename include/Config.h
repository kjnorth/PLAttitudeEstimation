/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 10:46:53 
 * @Desc: includes all files, holds pin definitions,
 * and holds #defines for the project 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "..\lib\Attitude\Attitude.h"
#include "..\lib\Attitude\AttitudeTest.h"
#include "..\lib\Adafruit_LSM9DS1\Adafruit_LSM9DS1.h"
#include "..\lib\DataLog\DataLog.h"
#include "IMU.h"
#include "..\lib\Matrix\Matrix.h"
#include "..\lib\Matrix\MatrixTest.h"

#define IMU_SAMPLE_TIME                         20 // ms
// accelerometer moving average
#define SHIFT_ACC 			                    3
#define SAMPLES_ACC 		                    (1 << SHIFT_ACC)
// accelerometer offsets and scale factor
#define ACCEL_X_OFFSET 			                -453.11
#define ACCEL_Y_OFFSET 			                -287.54
#define ACCEL_Z_OFFSET 			                -104.92
#define ACCEL_SCALE_FACTOR 	                    16393.44 // LSB/g
// gyro scale factor from the datasheet
#define GYRO_SF_DATASHEET                       0.00875 // 8.75 mdps/LSB
// gyro scale factor from calibration process
#define GYRO_SF_X_CALIBRATION                   180 / 8566.379
#define GYRO_SF_Y_CALIBRATION                   180 / 8863.956
#define GYRO_SF_Z_CALIBRATION                   180 / 8978.198
#define NUM_SAMPLES_NORMALIZE_GYRO_AVERAGE      100
#define NUM_GYRO_BIAS_SAMPLES                   100.0

#endif /* CONFIG_H */