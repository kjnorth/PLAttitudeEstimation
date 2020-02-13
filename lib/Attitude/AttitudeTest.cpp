/* 
 * File:   AttitudeTest.h
 * Author: Kodiak
 *
 * Created on March 9, 2019, 2:13 PM
 */

#include "AttitudeTest.h"

void AttitudeTests() {
    AttitudeDcmToEulerTest();
    AttitudeEulerToDcmTest();
    // AttitudeRexpTest();
    // test use of cos with small inputs .. cos becomes less accurate near 90 degrees,
    // but this is the known gimbal lock issue, and the PL should never get within
    // 10 degrees of 90
    // for (int i = 0; i < 5; i++) {
    //     static float testVal = 0.0001;
    //     Serial.print("test cos(");
    //     Serial.print(testVal, 5);
    //     Serial.print("): ");
    //     Serial.println(cos(testVal), 5);
    //     testVal = testVal * 10;
    // }
    // Serial.print("test cos(");
    // Serial.print(HALF_PI, 5);
    // Serial.print("): ");
    // Serial.println(AttitudeMaclaurinCos(HALF_PI), 20);
}

void AttitudeDcmToEulerTest() {
    // Test 1
    float dcm[3][3] = {
        .5, .5, -.7071,
        -.1464, .8536, .5,
        .8536, -.1464, .5
    };
    struct Euler angles;
    AttitudeEulerInit(&angles);
    AttitudeDcmToEuler(dcm, &angles);
    // expected angles in degrees
    float epsi = 45.0;
    float etheta = 45.0;
    float ephi = 45.0;
    float psiDiff = fabs(epsi - angles.psi*180/PI);
    float thetaDiff = fabs(etheta - angles.theta*180/PI);
    float phiDiff = fabs(ephi - angles.phi*180/PI);
    if(psiDiff < FP_DELTA2 && thetaDiff < FP_DELTA2 && phiDiff < FP_DELTA2) {
        Serial.print("AttitudeDcmToEulerTest 1: PASSED\n");
    } else {
        Serial.print("AttitudeDcmToEulerTest 1: FAILED\n");
        Serial.print("psi: ");
        Serial.print(angles.psi*(180/PI), 5);
        Serial.print(", theta: ");
        Serial.print(angles.theta*(180/PI), 5);
        Serial.print(", phi: ");
        Serial.println(angles.phi*(180/PI), 5);
        Serial.print("psiDiff: ");
        Serial.print(psiDiff, 7);
        Serial.print(", thetaDiff: ");
        Serial.print(thetaDiff, 7);
        Serial.print(", phiDiff: ");
        Serial.println(phiDiff, 7);
    }
    
    // Test 2
    float dcm2[3][3] = {
        0.4924,    0.4132,   -0.7660,
        0.1868,    0.8095,    0.5567,
        0.8501,   -0.4172,    0.3214
    };
    struct Euler angles2;
    AttitudeEulerInit(&angles2);
    AttitudeDcmToEuler(dcm2, &angles2);
    // expected angles in degrees
    float epsi2 = 40.0;
    float etheta2 = 50.0;
    float ephi2 = 60.0;
    float psiDiff2 = fabs(epsi2 - angles2.psi*180/PI);
    float thetaDiff2 = fabs(etheta2 - angles2.theta*180/PI);
    float phiDiff2 = fabs(ephi2 - angles2.phi*180/PI);
    if(psiDiff2 < FP_DELTA2 && thetaDiff2 < FP_DELTA2 && phiDiff2 < FP_DELTA2) {
        Serial.print("AttitudeDcmToEulerTest 2: PASSED\n");
    } else {
        Serial.print("AttitudeDcmToEulerTest 2: FAILED\n");
        Serial.print("psi2: ");
        Serial.print(angles2.psi*(180/PI), 5);
        Serial.print(", theta2: ");
        Serial.print(angles2.theta*(180/PI), 5);
        Serial.print(", phi2: ");
        Serial.println(angles2.phi*(180/PI), 5);
        Serial.print("psiDiff2: ");
        Serial.print(psiDiff2, 7);
        Serial.print(", thetaDiff2: ");
        Serial.print(thetaDiff2, 7);
        Serial.print(", phiDiff2: ");
        Serial.println(phiDiff2, 7);
    }

    // Test 3
    float dcm3[3][3] = {
        1.0000,    0.0087,   -0.0044,
        -0.0087,    1.0000,    0.0017,
        0.0044,   -0.0017,    1.0000
    };
    struct Euler angles3;
    AttitudeEulerInit(&angles3);
    AttitudeDcmToEuler(dcm3, &angles3);
    // expected angles in degrees
    float epsi3 = 0.5;
    float etheta3 = 0.25;
    float ephi3 = 0.1;
    float psiDiff3 = fabs(epsi3 - angles3.psi*180/PI);
    float thetaDiff3 = fabs(etheta3 - angles3.theta*180/PI);
    float phiDiff3 = fabs(ephi3 - angles3.phi*180/PI);
    if(psiDiff3 < FP_DELTA2 && thetaDiff3 < FP_DELTA2 && phiDiff3 < FP_DELTA2) {
        Serial.print("AttitudeDcmToEulerTest 3: PASSED\n");
    } else {
        Serial.print("AttitudeDcmToEulerTest 3: FAILED\n");
        Serial.print("psi3: ");
        Serial.print(angles3.psi*(180/PI), 5);
        Serial.print(", theta3: ");
        Serial.print(angles3.theta*(180/PI), 5);
        Serial.print(", phi3: ");
        Serial.println(angles3.phi*(180/PI), 5);
        Serial.print("psiDiff3: ");
        Serial.print(psiDiff3, 7);
        Serial.print(", thetaDiff3: ");
        Serial.print(thetaDiff3, 7);
        Serial.print(", phiDiff3: ");
        Serial.println(phiDiff3, 7);
    }
}

void AttitudeEulerToDcmTest() {
    // Test 1
    struct Euler angles;
    AttitudeEulerInit(&angles);
    angles.psi = 40*PI/180;
    angles.theta = 50*PI/180;
    angles.phi = 60*PI/180;
    float edcm[3][3] = {
        0.4924,    0.4132,   -0.7660,
        0.1868,    0.8095,    0.5567,
        0.8501,   -0.4172,    0.3214
    };
    float adcm[3][3] = {0.0};
    AttitudeEulerToDcm(&angles, adcm);
    int val = 0;
    val =  MatrixEquals(edcm, adcm);
    if(val)
        Serial.print("AttitudeEulerToDcmTest 1: PASSED\n");
    else
        Serial.print("AttitudeEulerToDcmTest 1: FAILED\n");
}

void AttitudeRexpTest() {
    // NOTE: this test increases w[2] way beyond the range it should ever be in
    int c = 0;
    float w[3] = {0.00001, 0.00001, 0.00001};
    float Rexp[3][3] = {0.0};
    for (int i = 0; i < 3; i++) {
        w[0] = w[0]*10;
        for (int j = 0; j < 3; j++) {
            w[1] = w[1]*10;
            for (int k = 0; k < 3; k++) {
                w[2] = w[2]*10;
                Serial.println(c++);
                Serial.println(MatrixVectorNorm(w), 4);
                AttitudeRexp(Rexp, w);
                MatrixPrint(Rexp);
            }
        }
    }
}
