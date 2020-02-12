/* 
 * File:   Matrix.c
 * Author: Kodiak
 *
 * Created on March 3, 2019, 7:16 PM
 */

#include "Matrix.h"

void MatrixMultiply(float m1[3][3], float m2[3][3], float res[3][3]) {
    // res row 0
    res[0][0] = m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0];
    res[0][1] = m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1];
    res[0][2] = m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2];
    // res row 1
    res[1][0] = m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0];
    res[1][1] = m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1];
    res[1][2] = m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2];
    // res row 2
    res[2][0] = m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0];
    res[2][1] = m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1];
    res[2][2] = m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2];
}

void MatrixAdd(float m1[3][3], float m2[3][3], float res[3][3]) {
    // res row 0
    res[0][0] = m1[0][0] + m2[0][0];
    res[0][1] = m1[0][1] + m2[0][1];
    res[0][2] = m1[0][2] + m2[0][2];
    // res row 1
    res[1][0] = m1[1][0] + m2[1][0];
    res[1][1] = m1[1][1] + m2[1][1];
    res[1][2] = m1[1][2] + m2[1][2];
    // res row 2
    res[2][0] = m1[2][0] + m2[2][0];
    res[2][1] = m1[2][1] + m2[2][1];
    res[2][2] = m1[2][2] + m2[2][2];
}

void MatrixVectorMultiply(float m[3][3], float v[3], float res[3]) {
    res[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    res[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    res[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}

void MatrixVectorAdd(float v1[3], float v2[3], float res[3]) {
    res[0] = v1[0] + v2[0];
    res[1] = v1[1] + v2[1];
    res[2] = v1[2] + v2[2];
}

void MatrixVectorSub(float v1[3], float v2[3], float res[3]) {
    res[0] = v1[0] - v2[0];
    res[1] = v1[1] - v2[1];
    res[2] = v1[2] - v2[2];
}

float MatrixVectorNorm(float v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int MatrixEquals(float m1[3][3], float m2[3][3]) {
    float difference = 0;
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            difference = fabs(m1[i][j] - m2[i][j]);
            if (difference > FP_DELTA) {
                return 0;
            }
        }
    }
    return 1;
}

void MatrixScalarMultiply(float x, float m[3][3], float res[3][3]) {
    // res row 0
    res[0][0] = x * m[0][0];
    res[0][1] = x * m[0][1];
    res[0][2] = x * m[0][2];
    // res row 1
    res[1][0] = x * m[1][0];
    res[1][1] = x * m[1][1];
    res[1][2] = x * m[1][2];
    // res row 2
    res[2][0] = x * m[2][0];
    res[2][1] = x * m[2][1];
    res[2][2] = x * m[2][2];
}

void MatrixScalarAdd(float x, float m[3][3], float res[3][3]) {
    // res row 0
    res[0][0] = x + m[0][0];
    res[0][1] = x + m[0][1];
    res[0][2] = x + m[0][2];
    // res row 1
    res[1][0] = x + m[1][0];
    res[1][1] = x + m[1][1];
    res[1][2] = x + m[1][2];
    // res row 2
    res[2][0] = x + m[2][0];
    res[2][1] = x + m[2][1];
    res[2][2] = x + m[2][2];
}

float MatrixDeterminant(float m[3][3]) {
    float det = 0;
    det = (m[0][0] * ((m[1][1] * m[2][2]) - (m[2][1] * m[1][2])))
            - (m[0][1] * ((m[1][0] * m[2][2]) - (m[2][0] * m[1][2])))
            + (m[0][2] * ((m[1][0] * m[2][1]) - (m[2][0] * m[1][1])));
    return det;
}

void MatrixTranspose(float m[3][3], float res[3][3]) {
    // res row 0
    res[0][0] = m[0][0];
    res[0][1] = m[1][0];
    res[0][2] = m[2][0];
    // res row 1
    res[1][0] = m[0][1];
    res[1][1] = m[1][1];
    res[1][2] = m[2][1];
    // res row 2
    res[2][0] = m[0][2];
    res[2][1] = m[1][2];
    res[2][2] = m[2][2];
}

void MatrixInverse(float m[3][3], float res[3][3]) {
    float det = MatrixDeterminant(m);
    if (det != 0) {
        float cofactor[3][3] = {};
        float detInv = 1 / det;

        cofactor[0][0] = (m[1][1] * m[2][2]) - (m[1][2] * m[2][1]);
        cofactor[1][0] = (m[1][2] * m[2][0]) - (m[1][0] * m[2][2]);
        cofactor[2][0] = (m[1][0] * m[2][1]) - (m[1][1] * m[2][0]);
        cofactor[0][1] = (m[0][2] * m[2][1]) - (m[0][1] * m[2][2]);
        cofactor[1][1] = (m[0][0] * m[2][2]) - (m[0][2] * m[2][0]);
        cofactor[2][1] = (m[0][1] * m[2][0]) - (m[0][0] * m[2][1]);
        cofactor[0][2] = (m[0][1] * m[1][2]) - (m[0][2] * m[1][1]);
        cofactor[1][2] = (m[0][2] * m[1][0]) - (m[0][0] * m[1][2]);
        cofactor[2][2] = (m[0][0] * m[1][1]) - (m[0][1] * m[1][0]);

        MatrixScalarMultiply(detInv, cofactor, res);
    }
}

void MatrixPrint(float m[3][3]) {
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            Serial.print("| ");
            Serial.print(m[i][j], 4);
            Serial.print(" ");
        }
        Serial.print("|\n");
        Serial.print("----------------------\n");
    }
}

void MatrixCopy(float m1[3][3], float m2[3][3]) {
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m2[i][j] = m1[i][j];
        }
    }
}