/* 
 * File:   Attitude.h
 * Author: Kodiak
 *
 * Created on March 9, 2019, 2:03 PM
 */

#ifndef ATTITUDE_H
#define	ATTITUDE_H

#include <Arduino.h>
#include <stdio.h>
#include <Math.h>
#include "..\Matrix\Matrix.h"

#define FP_DELTA2 0.004
// #define PI 3.14159265 // PI already defined in Arduion.h

// define struct for Euler angles in RADIANS
typedef struct Euler {
    float psi; // rot about Z // yaw
    float theta; // rot about Y // pitch
    float phi; // rot about X // roll
} Euler;

// initializes angles to 0
void AttitudeEulerInit(Euler* angles);

void AttitudePrintEuler(Euler* angles);

void AttitudePrintVector(float v[3]);

/**
 * 
 * @param x - angle in radians
 * @return the Maclaurin approximation of cos(x)
 */
float AttitudeMaclaurinCos(float x);

/**
 * Converts DCM to Euler angles
 * @param dcm - the direct cosine matrix
 * @param angles - euler angles in RADIANS
 */
void AttitudeDcmToEuler(float dcm[3][3], Euler* angles);

/**
 * Converts Euler angles into a DCM
 * @param angles
 * @param dcm
 */
void AttitudeEulerToDcm(Euler* angles, float dcm[3][3]);

/**
 * @param v - vector of x,y,z data. In this case it is from the gyro
 * @param rx - the skew symmetric x-product matrix of a 3x1 vector, v 
 */
void AttitudeRcross(float v[3], float rx[3][3]);

/**
 * 
 * @param Rexp - exponential Rodrigues parameter
 * @param w - gyro vector * deltaT
 */
void AttitudeRexp(float Rexp[3][3], float w[3]);

/**
 * 
 * @param R - the current DCM
 * @param nR - the new DCM ... this is essentially what is "returned"
 * @param g - vector of gyro data in RADIANS
 * @param deltaT - the change in time
 */
void AttitudeOpenLoopIntegration(float R[3][3], float nR[3][3], float g[3], float deltaT);

void AttitudeClosedLoopIntegrationAcc(float R[3][3], float nR[3][3], float gyro[3], float acc[3], float accI[3]);

void AttitudeClosedLoopIntegrationAccMag(float R[3][3], float nR[3][3], float gyro[3],
                                        float acc[3], float mag[3], float biasE[3],
                                        float accI[3], float magI[3], float dt);

void AttitudeDcmFromTriad(float nR[3][3], float acc[3], float mag[3], float accI[3], float magI[3]);

/**
 * 
 * @param raw - raw data
 * @param A - correction scale matrix
 * @param B - correction offset vector
 * @param corr - corrected data
 */
void AttitudeCorrectData(float raw[3], float A[3][3], float B[3], float corr[3]);

#endif	/* ATTITUDE_H */

