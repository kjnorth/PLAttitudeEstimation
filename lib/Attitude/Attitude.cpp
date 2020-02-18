/* 
 * File:   Attitude.c
 * Author: Kodiak
 *
 * Created on March 9, 2019, 2:03 PM
 */

#include "Attitude.h"

void AttitudeEulerInit(Euler* angles) {
    angles->psi = 0.0;
    angles->theta = 0.0;
    angles->phi = 0.0;
}

void AttitudePrintEuler(Euler* angles) {
    // printf("psi: %.5f, theta: %.5f, phi: %.5f\r\n",angles->psi,angles->theta,angles->phi);
    // Serial.print("yaw: ");
    // Serial.print(angles->psi*(180/PI), 3);
    // not estimating yaw now
    // Serial.print("euler pitch: ");
    // Serial.print(angles->theta*(180/PI), 3);
    // Serial.print(", roll: ");
    // Serial.print(-angles->phi*(180/PI), 3);
    // for data logging
    Serial.print(", ");
    Serial.print(angles->theta*(180/PI), 3);
    Serial.print(", ");
    Serial.print(-angles->phi*(180/PI), 3);
    Serial.print(", ");
    Serial.println(millis());
}

void AttitudePrintVector(float v[3]) {
    // printf("%.5f; %.5f; %.5f\r\n", v[0], v[1], v[2]);
    Serial.print(v[0], 5);
    Serial.print(", ");
    Serial.print(v[1], 5);
    Serial.print(", ");
    Serial.println(v[2], 5);
}

float AttitudeMaclaurinCos(float x) {
    float cos_estimate = 1 - (powf(x, 2) / 2) + (powf(x, 4)/24) - (powf(x, 6) / 720) + (powf(x, 8) / 40320) - (powf(x, 10)/3628800);
    return cos_estimate;
}

void AttitudeDcmToEuler(float dcm[3][3], Euler* angles) {
    angles->theta = asin(-1.0*dcm[0][2]); // pitch
    angles->psi = atan2(dcm[0][1], dcm[0][0]); // yaw
    angles->phi = atan2(dcm[1][2], dcm[2][2]); // roll
}

void AttitudeEulerToDcm(Euler* angles, float dcm[3][3]) {
    dcm[0][0] = cos(angles->theta)*cos(angles->psi);
    dcm[0][1] = cos(angles->theta)*sin(angles->psi);
    dcm[0][2] = -sin(angles->theta);
    dcm[1][0] = sin(angles->phi)*sin(angles->theta)*cos(angles->psi)-cos(angles->phi)*sin(angles->psi);
    dcm[1][1] = sin(angles->phi)*sin(angles->theta)*sin(angles->psi)+cos(angles->phi)*cos(angles->psi);
    dcm[1][2] = sin(angles->phi)*cos(angles->theta);
    dcm[2][0] = cos(angles->phi)*sin(angles->theta)*cos(angles->psi)+sin(angles->phi)*sin(angles->psi);
    dcm[2][1] = cos(angles->phi)*sin(angles->theta)*sin(angles->psi)-sin(angles->phi)*cos(angles->psi);
    dcm[2][2] = cos(angles->phi)*cos(angles->theta);
}

// v is a vector of gyro data
void AttitudeRcross(float v[3], float rx[3][3]) {
    rx[0][0] = 0.0;
    rx[0][1] = -1.0*v[2];
    rx[0][2] = v[1];
    rx[1][0] = v[2];
    rx[1][1] = 0.0;
    rx[1][2] = -1.0*v[0];
    rx[2][0] = -1.0*v[1];
    rx[2][1] = v[0];
    rx[2][2] = 0.0;
}

void AttitudeRexp(float Rexp[3][3], float w[3]) {
    float wnorm = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    float rx[3][3] = {0.0};
    AttitudeRcross(w, rx);
    float s = 0.0;
    float x = wnorm/2.0;
    if (wnorm > 0.15) {
        s = sin(x)/x;
    } else {
        // must estimate sinc because division by apprx 0 will break code
        s = 1 - x*x/6 + x*x*x*x/120 - x*x*x*x*x*x/5040 + x*x*x*x*x*x*x*x/362880;
    }
    float c = cos(x);
    float s_c_rx[3][3] = {0.0}; // s*c*rx
    MatrixScalarMultiply(s*c, rx, s_c_rx);
    float rx_2[3][3] = {0.0}; // rx^2
    MatrixMultiply(rx, rx, rx_2);
    float ssD2rx_2[3][3] = {0.0}; // s*s/2*rx*rx
    MatrixScalarMultiply(s*s/2.0, rx_2, ssD2rx_2);
    float res[3][3] = {0.0};
    MatrixAdd(s_c_rx, ssD2rx_2, res);
    // treat x as a new variable
    float I[3][3] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    MatrixAdd(I, res, Rexp);
}

// make sure g[0-2] are passed in as RADIANS
void AttitudeOpenLoopIntegration(float R[3][3], float nR[3][3], float g[3], float deltaT) {
    // forward integration
    /*
    float rx[3][3] = {0.0};
    AttitudeRcross(g, rx);
    float rx_dt[3][3] = {0.0}; // rx*dt
    MatrixScalarMultiply(deltaT, rx, rx_dt);
    float res[3][3] = {0.0};
    MatrixMultiply(R, rx_dt, res);
    MatrixAdd(R, res, nR);
    //*/

    // matrix exponential
    float Rexp[3][3] = {0.0};
    float w[3] = {0.0};
//    AttitudeEulerInit(&w);
    w[0] = g[0]*deltaT;
    w[1] = g[1]*deltaT;
    w[2] = g[2]*deltaT;
    AttitudeRexp(Rexp, w);
    MatrixMultiply(R, Rexp, nR);
}

void AttitudeClosedLoopIntegrationAcc(float R[3][3], float nR[3][3], float gyro[3], float acc[3], float accI[3]) {
    static float biasE[3] = {0.0}; // running bias estimate
    unsigned long curT = millis();
    static unsigned long preT = curT;
    float dt = (curT - preT) / 1000.0; // dt in seconds
    preT = curT;
    // tuning parameters
    float akp = 4.5;
    float aki = 0.009783925;
    // get wmeas_a
    float RT[3][3] = {0.0};
    MatrixTranspose(R, RT);
    float RT_accI[3] = {0.0}; // R' * accelInertial
    MatrixVectorMultiply(RT, accI, RT_accI);
    // acc data * dt
    float acc_dt[3] = {dt*acc[0], dt*acc[1], dt*acc[2]};
    float rxa[3][3] = {0.0}; // rcross matrix created with accel data
    AttitudeRcross(acc_dt, rxa);
    float wa[3] = {0.0};
    MatrixVectorMultiply(rxa, RT_accI, wa);
    
    // gyro data * dt
    float gyro_dt[3] = {dt*gyro[0], dt*gyro[1], dt*gyro[2]};
    // get gyro data with feedback
    float akp_wa[3] = {akp*wa[0], akp*wa[1], akp*wa[2]}; // akp * wa
    float gyro_bE[3] = {0.0}; // gyro input MINUS biasEstimate
    MatrixVectorSub(gyro_dt, biasE, gyro_bE);
    float gyroWithFb[3] = {0.0}; // gyro input with feedback
    MatrixVectorAdd(gyro_bE, akp_wa, gyroWithFb);
    // get Rexp from gyroWithFb
    float Rexp[3][3] = {0.0};
    AttitudeRexp(Rexp, gyroWithFb);
    // calculate new R
    MatrixMultiply(R, Rexp, nR);
    
    // get the delta bias (derivative)
    float bdot[3] = {-aki*wa[0], -aki*wa[1], -aki*wa[2]};
    // add delta bias to biasEstimate
    MatrixVectorAdd(biasE, bdot, biasE);
}

void AttitudeClosedLoopIntegrationAccMag(float R[3][3], float nR[3][3], float gyro[3],
                                        float acc[3], float mag[3], float biasE[3],
                                        float accI[3], float magI[3], float dt) {
    // tuning parameters
    float akp = .9;
    float aki = 0.18;
    float mkp = .03;
    float mki = .0095;
    // set acc, mag, and gyro input data to unit vectors
    float accNorm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    acc[0] = acc[0]/accNorm;
    acc[1] = acc[1]/accNorm;
    acc[2] = acc[2]/accNorm;
    float magNorm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    mag[0] = mag[0]/magNorm;
    mag[1] = mag[1]/magNorm;
    mag[2] = mag[2]/magNorm;
    
    // get wmeas_a
    float RT[3][3] = {0.0};
    MatrixTranspose(R, RT);
    float RT_accI[3] = {0.0}; // R' * accel inertial
    MatrixVectorMultiply(RT, accI, RT_accI);
    // compute Rcross matrix with acc data
    float rxa[3][3] = {0.0};
    AttitudeRcross(acc, rxa);
    float wa[3] = {0.0};
    MatrixVectorMultiply(rxa, RT_accI, wa);
    
    // get wmeas_m
    float RT_magI[3] = {0.0}; // R' * mag inertial
    MatrixVectorMultiply(RT, magI, RT_magI);
    // compute Rcross matrix with mag data
    float rxm[3][3] = {0.0};
    AttitudeRcross(mag, rxm);
    float wb[3] = {0.0};
    MatrixVectorMultiply(rxm, RT_magI, wb);
    
    // gyro data * dt
    float gyro_dt[3] = {dt*gyro[0], dt*gyro[1], dt*gyro[2]};
    // get gyro data with feedback
    float akp_wa[3] = {akp*wa[0], akp*wa[1], akp*wa[2]}; // akp * wa
    float mkp_wm[3] = {mkp*wb[0], mkp*wb[1], mkp*wb[2]}; // mkp * wb
    float gyro_bE[3] = {0.0}; // gyro input MINUS biasEstimate
    MatrixVectorSub(gyro_dt, biasE, gyro_bE);
    float akpwa_mkpwm[3] = {0.0}; // akp*wa + mkp*wm
    MatrixVectorAdd(akp_wa, mkp_wm, akpwa_mkpwm);
    float gyroWithFb[3] = {0.0}; // gyro input with feedback - gyro_bE + akp*wa + mkp*wm
    MatrixVectorAdd(gyro_bE, akpwa_mkpwm, gyroWithFb);
    // get Rexp from gyroWithFb
    float Rexp[3][3] = {0.0};
    AttitudeRexp(Rexp, gyroWithFb);
    // calculate new R
    MatrixMultiply(R, Rexp, nR);
    
    // get the delta bias (derivative)
    float aki_wa_dot[3] = {-aki*wa[0], -aki*wa[1], -aki*wa[2]};
    float mki_wm_dot[3] = {mki*wb[0], mki*wb[1], mki*wb[2]};
    float bdot[3] = {0.0};
    MatrixVectorSub(aki_wa_dot, mki_wm_dot, bdot);
    // add delta bias to biasEstimate
    MatrixVectorAdd(biasE, bdot, biasE);
}

void AttitudeDcmFromTriad(float nR[3][3], float acc[3], float mag[3], float accI[3], float magI[3]) {
    // proves the algorithm works
    /*
    accI[0] = 0;
    accI[1] = 0;
    accI[2] = -1;
    acc[0] = 0;
    acc[1] = 0;
    acc[2] = -1;
    magI[0] = 1;
    magI[1] = 0;
    magI[2] = 0;
    mag[0] = 1;
    mag[1] = 0;
    mag[2] = 0;
    */
    
//    float Ro[3][3] = {
//        {1.0, 0.0, 0.0},
//        {0.0, 1.0, 0.0},
//        {0.0, 0.0, 1.0}
//    };
//    MatrixVectorMultiply(Ro, acc, acc);
//    MatrixVectorMultiply(Ro, mag, mag);
    
    // accI and magI already have unit length
    // convert acc and mag readings to unit length
    float accNorm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    // calculate acc in unit length
    acc[0] = acc[0]/accNorm;
    acc[1] = acc[1]/accNorm;
    acc[2] = acc[2]/accNorm;
    // get norm of mag
    float magNorm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    // calculate mag in unit length
    mag[0] = mag[0]/magNorm;
    mag[1] = mag[1]/magNorm;
    mag[2] = mag[2]/magNorm;
    
    // compute Rcross matrix with mag data
    float rxm[3][3] = {0.0};
    AttitudeRcross(mag, rxm);
    // compute m
    float m[3] = {0.0};
    MatrixVectorMultiply(rxm, acc, m);
    float mNorm = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    // convert m to unit length
    m[0] = m[0]/mNorm;
    m[1] = m[1]/mNorm;
    m[2] = m[2]/mNorm;
    
    // compute Rcross matrix with magInertial data
    float rxM[3][3] = {0.0};
    AttitudeRcross(magI, rxM);
    // compute M
    float M[3] = {0.0};
    MatrixVectorMultiply(rxM, accI, M);
    float MNorm = sqrt(M[0]*M[0] + M[1]*M[1] + M[2]*M[2]);
    // convert M to unit length
    M[0] = M[0]/MNorm;
    M[1] = M[1]/MNorm;
    M[2] = M[2]/MNorm;
    
    float rxMM[3] = {0.0}; // Rcross(magInertial)*M
    MatrixVectorMultiply(rxM, M, rxMM);
    float a1[3][3] = {
        {magI[0], M[0], rxMM[0]},
        {magI[1], M[1], rxMM[1]},
        {magI[2], M[2], rxMM[2]}
    };
    
    float rxmm[3] = {0.0}; // Rcross(mag)*m
    MatrixVectorMultiply(rxm, m, rxmm);
    float a2[3][3] = {
        {mag[0], m[0], rxmm[0]},
        {mag[1], m[1], rxmm[1]},
        {mag[2], m[2], rxmm[2]}
    };
    float a2T[3][3] = {0.0};
    MatrixTranspose(a2, a2T); // transpose a2
    float A[3][3] = {0.0};
    MatrixMultiply(a1, a2T, A);
    
    MatrixTranspose(A, nR);
}

#define ALPHA 0.95//0.85
void AttitudeComplimentaryFilter(float gyro[3], float acc[3], Euler* angles) {
    unsigned long curTime = millis();
    static unsigned long preTime = curTime;
    float dt = (curTime - preTime) / 1000.0; // dt in seconds
    preTime = curTime;

    angles->theta += gyro[1] * dt; // pitch is rotation about y-axis
    angles->phi += gyro[0] * dt; // roll is rotation about x-axis

    float pitchAcc = atan2f(acc[0], acc[2]) * 180/PI;
    float rollAcc = atan2f(acc[1], acc[2]) * 180/PI;

    angles->theta = angles->theta * ALPHA + pitchAcc * (1 - ALPHA);
    angles->phi = angles->phi * ALPHA + rollAcc * (1 - ALPHA);
}

// can generate scale matrices and offset vectors using Matlab, pass them into
// this function, and will map data into +/-1 range; rather than finding offsets
// and scale factors for each axis
void AttitudeCorrectData(float raw[3], float A[3][3], float B[3], float corr[3]) {
    float scaleData[3] = {0.0};
    MatrixVectorMultiply(A, raw, scaleData);
    MatrixVectorAdd(scaleData, B, corr);
}