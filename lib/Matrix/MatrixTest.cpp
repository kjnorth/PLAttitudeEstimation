/* 
 * File:   MatrixTest.h
 * Author: Kodiak
 *
 * Created on March 4, 2019, 10:29 AM
 */

#include "MatrixTest.h"

// matrices used for testing
float mat1[3][3] = {
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8}
};
float mat2[3][3] = {
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8}
};
float mat3[3][3] = {
    {0, 1, 3},
    {3, -4, 5},
    {6, 7.1, 8}
};
float mat4[3][3] = {
    {25, 1, 37},
    {3, -4, 8.324},
    {1.77, 7.5, 8}
};
float mat5[3][3] = {
    {1.1111, 1, 5},
    {-30, 4, 5.7},
    {6, 7.88, 8}
};
float mat6[3][3] = {
    {-200, 50, 4.1234},
    {89, 0.45, 9.99},
    {1.47, -8.2, -5.8}
};
float result[3][3];

// calls all functions in the MatrixTest library
void MatrixTests(void) {
    Serial.print("Matrix Tests:\n");
    MatrixPrintTest();
    MatrixEqualsTest();
    MatrixAddTest();
    MatrixScalarMultiplyTest();
    MatrixScalarAddTest();
    MatrixMultiplyTest();
    MatrixTransposeTest();
    MatrixDeterminantTest();
    MatrixInverseTest();
    MatrixCopyTest();
}

void MatrixMultiplyTest() {
    // Test 1
    float expected[3][3] = {
        {15, 10.2, 21},
        {42, 22.5, 69},
        {69, 34.8, 117}
    };
    MatrixMultiply(mat1, mat3, result);
    int val = 0;
    val = MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixMultiplyTest 1: PASSED\n");
    else
        Serial.print("MatrixMultiplyTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {219.7775, 320.56, 426.7},
        {173.2773, 52.59312, 58.792},
        {-175.033353, 94.81, 115.6}
    };
    MatrixMultiply(mat4, mat5, result);

    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixMultiplyTest 2: PASSED\n");
    else
        Serial.print("MatrixMultiplyTest 2: FAILED\n");
}

void MatrixAddTest() {
    // Test 1
    float expected[3][3] = {
        {25, 2, 40},
        {6, -8, 13.324},
        {7.77, 14.6, 16}
    };
    MatrixAdd(mat3, mat4, result);
    int val = 0;
    val = MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixAddTest 1: PASSED\n");
    else
        Serial.print("MatrixAddTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {26.1111, 2, 42},
        {-27, 0, 14.024},
        {7.77, 15.38, 16}
    };
    MatrixAdd(mat4, mat5, result);

    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixAddTest 2: PASSED\n");
    else
        Serial.print("MatrixAddTest 2: FAILED\n");
}

void MatrixEqualsTest() {
    // Test 1 for success
    int expected = 1;
    int testResult = MatrixEquals(mat1, mat2);

    if (expected == testResult) {
        Serial.print("MatrixEqualsTest 1: PASSED\n");
    } else {
        Serial.print("MatrixEqualsTest 1: FAILED\n");
    }
    // Test 2 for failure
    expected = 0;
    testResult = MatrixEquals(mat1, mat3);

    if (expected == testResult) {
        Serial.print("MatrixEqualsTest 2: PASSED\n");
    } else {
        Serial.print("MatrixEqualsTest 2: FAILED\n");
    }
}

void MatrixScalarMultiplyTest() {
    // Test 1
    float expected[3][3] = {
        {0, 45.2, 135.6},
        {135.6, -180.8, 226},
        {271.2, 320.92, 361.6}
    };
    MatrixScalarMultiply(45.2, mat3, result);
    int val = 0;
    val =  MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixScalarMultiplyTest 1: PASSED\n");
    else
        Serial.print("MatrixScalarMultiplyTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {-7.7777, -7, -35},
        {210, -28, -39.9},
        {-42, -55.16, -56}
    };
    MatrixScalarMultiply(-7, mat5, result);

    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixScalarMultiplyTest 2: PASSED\n");
    else
        Serial.print("MatrixScalarMultiplyTest 2: FAILED\n");
}

void MatrixScalarAddTest() {
    // Test 1
    float expected[3][3] = {
        {-9.99, -8.99, -7.99},
        {-6.99, -5.99, -4.99},
        {-3.99, -2.99, -1.99}
    };
    MatrixScalarAdd(-9.99, mat1, result);
    int val = 0;
    val = MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixScalarAddTest 1: PASSED\n");
    else
        Serial.print("MatrixScalarAddTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {96, 72, 108},
        {74, 67, 79.324},
        {72.77, 78.5, 79}
    };
    MatrixScalarAdd(71, mat4, result);

    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixScalarAddTest 2: PASSED\n");
    else
        Serial.print("MatrixScalarAddTest 2: FAILED\n");
}

void MatrixDeterminantTest() {
    // Test 1
    float expected = -1275.556884;
    float testResult = MatrixDeterminant(mat4);
    if (expected == testResult) {
        Serial.print("MatrixDeterminantTest 1: PASSED\n");
    } else {
        Serial.print("MatrixDeterminantTest 1: FAILED\n");
    }
    // Test 2
    float expected2 = 7670.680664;
    float testResult2 = MatrixDeterminant(mat6);

    if (expected2 == testResult2) {
        Serial.print("MatrixDeterminantTest 2: PASSED\n");
    } else {
        Serial.print("MatrixDeterminantTest 2: FAILED\n");
    }
}

void MatrixTransposeTest() {
    // Test 1
    float expected[3][3] = {
        {25, 3, 1.77},
        {1, -4, 7.5},
        {37, 8.324, 8}
    };
    MatrixTranspose(mat4, result);
    int val = 0;
    val = MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixTransposeTest 1: PASSED\n");
    else
        Serial.print("MatrixTransposeTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {25, 3, 1.77},
        {1, -4, 7.5},
        {37, 8.324, 8}
    };
    MatrixTranspose(mat4, result);

    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixTransposeTest 2: PASSED\n");
    else
        Serial.print("MatrixTransposeTest 2: FAILED\n");
}

void MatrixInverseTest() {
    // Test 1
    float expected[3][3] = {
        {-0.4756871035940806, 0.09372797744890765, 0.11980267794221283},
        {0.04228329809725162, -0.12684989429175475, 0.06342494714587738},
        {0.3192389006342495, 0.042283298097251586, -0.021141649048625793}
    };

    MatrixInverse(mat3, result);
    int val = MatrixEquals(expected, result);
    if(val)
        Serial.print("MatrixInverseTest 1: PASSED\n");
    else
        Serial.print("MatrixInverseTest 1: FAILED\n");
    // Test 2
    float expected2[3][3] = {
        {0.07403043183064911, -0.2112803280563373, -0.12255356587413312},
        {0.007264687886978146, -0.1054520108603263, 0.07612363582289557},
        {-0.023189877936573126, 0.1456070327640205, 0.08074906786568736}
    };

    MatrixInverse(mat4, result);
    val = MatrixEquals(expected2, result);
    if(val)
        Serial.print("MatrixInverseTest 2: PASSED\n");
    else
        Serial.print("MatrixInverseTest 2: FAILED\n");
}

void MatrixPrintTest() {
    Serial.print("MatrixPrintTest prints mat1:\n");
    MatrixPrint(mat1);
}

void MatrixCopyTest() {
    float res[3][3] = {0.0};
    MatrixCopy(mat1, res);
    int val = MatrixEquals(mat1, res);
    if(val)
        Serial.print("MatrixCopyTest: PASSED\n");
    else
        Serial.print("MatrixCopyTest: FAILED\n");
}