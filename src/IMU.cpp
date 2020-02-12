/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 12:22:59 
 * @Desc: library to read and calibrate 
 * data from the LSM9DS1  IMU
 */

#include "IMU.h"

static Adafruit_LSM9DS1 dof = Adafruit_LSM9DS1();

float gyroXBias = 0.0, gyroYBias = 0.0, gyroZBias = 0.0;

bool IMU_Init(void) {
    if (!dof.begin()) {
		LogInfo("Failed to initialize IMU\n");
		return false;
	}
	LogInfo("IMU init complete\n");
    return true;
}

void IMU_Read(float acc[3], float gyro[3]) {
    dof.readAccel();
    acc[0] = dof.accelData.x;
    acc[1] = dof.accelData.y;
    acc[2] = dof.accelData.z;

    dof.readGyro();
    gyro[0] = dof.gyroData.x;
    gyro[1] = dof.gyroData.y;
    gyro[2] = dof.gyroData.z;
}


#define ACCEL_X_OFFSET 			-453.11
#define ACCEL_Y_OFFSET 			-287.54
#define ACCEL_Z_OFFSET 			-104.92
#define ACCEL_SCALE_FACTOR 	    16393.44 // LSB/g

/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 12:55:06 
 * @Desc: pass this function raw accelerometer data
 * to filter and map into the range of +/- 1g 
 */
void IMU_AccCalibrate(float acc[3]) {
    // scale to +/- 1g
    acc[0] = (acc[0] - ACCEL_X_OFFSET) / ACCEL_SCALE_FACTOR;
    acc[1] = (acc[1] - ACCEL_Y_OFFSET) / ACCEL_SCALE_FACTOR;
    acc[2] = (acc[2] - ACCEL_Z_OFFSET) / ACCEL_SCALE_FACTOR;
    // moving average
    static uint8_t index = 0;
    static float xsum = 0.0, ysum = 0.0, zsum = 0.0;
    static float xarr[SAMPLES_ACC] = {0.0};
    static float yarr[SAMPLES_ACC] = {0.0};
    static float zarr[SAMPLES_ACC] = {0.0};

    xsum -= xarr[index]; xarr[index] = acc[0]; xsum += xarr[index];
    ysum -= yarr[index]; yarr[index] = acc[1]; ysum += yarr[index];
    zsum -= zarr[index]; zarr[index] = acc[2]; zsum += zarr[index];
    index = (index + 1) & (SAMPLES_ACC - 1); // AND operation for binary modulo is faster than using %

    // muliply sum by 10000 to preserve 4 decimals when performing
	// shift operation, then divide by 10000.0 to return back to 
	// original units
	acc[0] = (float)((int)(xsum * 1000) >> SHIFT_ACC) / 1000.0;
    acc[1] = (float)((int)(ysum * 1000) >> SHIFT_ACC) / 1000.0;
    acc[2] = (float)((int)(zsum * 1000) >> SHIFT_ACC) / 1000.0;
}

void IMU_GyroCalibrate(float gyro[3]) {
    gyro[0] = (gyro[0] - gyroXBias) * GYRO_SCALE_FACTOR;
    gyro[1] = (gyro[1] - gyroYBias) * GYRO_SCALE_FACTOR;
    gyro[2] = (gyro[2] - gyroZBias) * GYRO_SCALE_FACTOR;
}

void IMU_CalculateGyroBias(void) {
    LogInfo("Calculating gyro biases for 10 seconds\n");

	float xSumG = 0.0, ySumG = 0.0, zSumG = 0.0; // gyro data
    for (int i = 0; i < NUM_GYRO_BIAS_SAMPLES; i++) {
		dof.readGyro();
		float xRawG = dof.gyroData.x; // roll
		float yRawG = dof.gyroData.y; // pitch
		float zRawG = dof.gyroData.z; // yaw
		xSumG += xRawG;
		ySumG += yRawG;
		zSumG += zRawG;
        delay(100);
    }
    gyroXBias = xSumG / NUM_GYRO_BIAS_SAMPLES;
	gyroYBias = ySumG / NUM_GYRO_BIAS_SAMPLES;
	gyroZBias = zSumG / NUM_GYRO_BIAS_SAMPLES;
    LogInfo("gyro x bias ", gyroXBias, 2);
    LogInfo(", gyro y bias ", gyroYBias, 2);
    LogInfo(", gyro z bias ", gyroZBias, 2, true);
}

void IMU_PrintData(float acc[3], float gyro[3]) {
    // prints data in CSV format for easy exporting
    uint8_t decimals = 3;
    // cannot add text because matlab doesn't like it.. maybe use python
    LogInfo("acc x, ", acc[0], decimals);
    LogInfo(", acc y, ", acc[1], decimals);
    LogInfo(", acc z, ", acc[2], decimals);
    LogInfo(", gyro x, ", gyro[0], decimals);
    LogInfo(", gyro y, ", gyro[1], decimals);
    LogInfo(", gyro z, ", gyro[2], decimals);
    LogInfo(F(", %lu\n"), millis());
}

void IMU_PrintAccPitchRoll(float acc[3]) {
    // prints data in CSV format for easy exporting
    float x = acc[0];
    float y = acc[1];
    float z = acc[2];
    float pitch = atan2(x, sqrt(y*y + z*z)) * 180/PI;
    float roll = -atan2(y, z) * 180/PI;
    LogInfo("pitch, ", pitch, 2);
    LogInfo(", roll, ", roll, 2);
    LogInfo(", acc x, ", acc[0], 4);
    LogInfo(", acc y, ", acc[1], 4);
    LogInfo(", acc z, ", acc[2], 4, true);
}