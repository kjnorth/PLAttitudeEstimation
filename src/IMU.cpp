/** 
 * @Author: Kodiak 
 * @Date: 2020-02-12 12:22:59 
 * @Desc: library to read and calibrate 
 * data from the LSM9DS1  IMU
 */

#include "IMU.h"

#define MAG_ADDRESS_LEFT    0x1E
#define XG_ADDRESS_LEFT     0x6B
#define MAG_ADDRESS_RIGHT   0x1C
#define XG_ADDRESS_RIGHT    0x6A 

TwoWire bus = TwoWire();
static Adafruit_LSM9DS1 dof = Adafruit_LSM9DS1(&bus);

float gyroXBias = 0.0, gyroYBias = 0.0, gyroZBias = 0.0;

bool IMU_Init(void) {
    /* Dual IMU's */
    // if (!dof.myBegin(MAG_ADDRESS_LEFT, XG_ADDRESS_LEFT, MAG_ADDRESS_RIGHT, XG_ADDRESS_RIGHT)) {
	// 	LogInfo("Failed to initialize IMU's\n");
	// 	return false;
	// }
    // LogInfo("IMU's initialized successfully\n");
    // return true;

    /* Single IMU */
    if (!dof.begin()) {
        LogInfo("Failed to initialize IMU\n");
        return false;
    }
	LogInfo("IMU initialized successfully\n");
    return true;
}

#define PL 0
#define TRUCK !PL
void IMU_Read(float acc[3], float gyro[3]) {
#if PL
    dof.readAccel();
    acc[0] = dof.accelData.x;
    // negate y to convert sensor data to
    // right handed coordinate system
    acc[1] = -dof.accelData.y;
    acc[2] = dof.accelData.z;

    dof.readGyro();
    gyro[0] = dof.gyroData.x;
    gyro[1] = dof.gyroData.y;
    gyro[2] = dof.gyroData.z;
#elif TRUCK
    dof.readAccel();
	acc[0] = dof.accelData.x;
	// y axis in right handed coordinate system since IMU is 
	// upside down on truck
	acc[1] = dof.accelData.y;
	// negate accel z since IMU mounted upside down on truck
	acc[2] = -dof.accelData.z;

	dof.readGyro();
	gyro[0] = dof.gyroData.x;
	// negate gyro y since IMU is upside down on truck
	gyro[1] = -dof.gyroData.y;
	gyro[2] = dof.gyroData.z;
#endif

    // test code for reads from two IMUs on same bus
    // read left accel
    // dof.myReadAccel(XG_ADDRESS_LEFT);
    // LogInfo("left acc z ", dof.accelData.z, 2);
    // // delay(10);

    // // read right accel
    // dof.myReadAccel(XG_ADDRESS_RIGHT);
    // LogInfo(" right acc z ", dof.accelData.z, 2, true);
}

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
    static float xarr[SAMPLES_ACCEL] = {0.0};
    static float yarr[SAMPLES_ACCEL] = {0.0};
    static float zarr[SAMPLES_ACCEL] = {0.0};

    xsum -= xarr[index]; xarr[index] = acc[0]; xsum += xarr[index];
    ysum -= yarr[index]; yarr[index] = acc[1]; ysum += yarr[index];
    zsum -= zarr[index]; zarr[index] = acc[2]; zsum += zarr[index];
    index = (index + 1) & (SAMPLES_ACCEL - 1); // AND operation for binary modulo is faster than using %

    // muliply sum by 1000 to preserve 3 decimals when performing
	// shift operation, then divide by 1000.0 to return back to 
	// original units
	acc[0] = (float)((int)(xsum * 1000) >> SHIFT_ACCEL) / 1000.0;
    acc[1] = (float)((int)(ysum * 1000) >> SHIFT_ACCEL) / 1000.0;
    acc[2] = (float)((int)(zsum * 1000) >> SHIFT_ACCEL) / 1000.0;
}

void IMU_GyroCalibrateDPS(float gyro[3]) {
    // subtract the initial bias term and convert to degs/s
    gyro[0] = (gyro[0] - gyroXBias) * GYRO_SF_DATASHEET * GYRO_SF_X_CALIBRATION;
    gyro[1] = (gyro[1] - gyroYBias) * GYRO_SF_DATASHEET * GYRO_SF_Y_CALIBRATION;
    gyro[2] = (gyro[2] - gyroZBias) * GYRO_SF_DATASHEET * GYRO_SF_Z_CALIBRATION;
}

void IMU_GyroCalibrateRPS(float gyro[3]) {
    // subtract the initial bias term and convert to rads/s
    gyro[0] = (gyro[0] - gyroXBias) * GYRO_SF_DATASHEET * GYRO_SF_X_CALIBRATION * PI/180;
    gyro[1] = (gyro[1] - gyroYBias) * GYRO_SF_DATASHEET * GYRO_SF_Y_CALIBRATION * PI/180;
    gyro[2] = (gyro[2] - gyroZBias) * GYRO_SF_DATASHEET * GYRO_SF_Z_CALIBRATION * PI/180;
}

void IMU_CalculateGyroBiasAndAccInertial(float accI[3]) {
    LogInfo("Calculating gyro biases for 10 seconds\n");

	float xSumG = 0.0, ySumG = 0.0, zSumG = 0.0; // gyro data
    float calA[3] = {0.0}; // calibrated accel data
    float xSumA = 0.0, ySumA = 0.0, zSumA = 0.0; // accel data
    for (int i = 0; i < NUM_GYRO_BIAS_SAMPLES; i++) {
		dof.readGyro();
		float xRawG = dof.gyroData.x; // roll
		float yRawG = dof.gyroData.y; // pitch
		float zRawG = dof.gyroData.z; // yaw
		xSumG += xRawG;
		ySumG += yRawG;
		zSumG += zRawG;

        dof.readAccel();
        // data is raw at this point
        calA[0] = (dof.accelData.x);
        // negate y to convert sensor data to
        // right handed coordinate system
        calA[1] = -(dof.accelData.y);
        calA[2] = (dof.accelData.z);
        IMU_AccCalibrate(calA); // now the data is calibrated
        xSumA += calA[0];
        ySumA += calA[1];
        zSumA += calA[2];

        delay(100);
    }
    // average gyro bias over the 10 second interval
    gyroXBias = xSumG / NUM_GYRO_BIAS_SAMPLES;
	gyroYBias = ySumG / NUM_GYRO_BIAS_SAMPLES;
	gyroZBias = zSumG / NUM_GYRO_BIAS_SAMPLES;
    // average acc data over the 10 second interval
    float xAvgA = xSumA / NUM_GYRO_BIAS_SAMPLES;
    float yAvgA = ySumA / NUM_GYRO_BIAS_SAMPLES;
    float zAvgA = zSumA / NUM_GYRO_BIAS_SAMPLES;

    // normal of calibrated acc data
    float accNorm = sqrt(xAvgA*xAvgA + yAvgA*yAvgA + zAvgA*zAvgA);
    // calculate inertial acc in unit length
    accI[0] = xAvgA / accNorm;
    accI[1] = yAvgA / accNorm;
    accI[2] = zAvgA / accNorm;
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
    LogInfo(F(", time, %lu\n"), millis());
}

void IMU_PrintAccData(float acc[3]) {
    uint8_t decimals = 3;
    // negate here so standard acc data can be used for
    // closed loop integration
    LogInfo("", acc[0], decimals);
    LogInfo(", ", acc[1], decimals);
    LogInfo(", ", acc[2], decimals);
    LogInfo(F(", %lu\n"), millis());
}

void IMU_PrintGyroData(float gyro[3]) {
    uint8_t decimals = 3;
    LogInfo("", gyro[0], decimals);
    LogInfo(", ", gyro[1], decimals);
    LogInfo(", ", gyro[2], decimals);
    LogInfo(F(", %lu\n"), millis());
}

void IMU_PrintAccPitchRoll(float acc[3]) {
    // prints data in CSV format for easy exporting
    float x = acc[0];
    float y = acc[1];
    float z = acc[2];
    // float pitch = atan2(x, sqrt(y*y + z*z)) * 180/PI;
    // float roll = atan2(y, -z) * 180/PI;
    float pitch = atan2f(x, z) * 180/PI;
    float roll = atan2f(y, z) * 180/PI;
    LogInfo(", acc pitch, ", pitch, 2);
    LogInfo(", roll, ", roll, 2);
    LogInfo(", acc x, ", x, 3);
    LogInfo(", acc y, ", y, 3);
    LogInfo(", acc z, ", z, 3, true);
}