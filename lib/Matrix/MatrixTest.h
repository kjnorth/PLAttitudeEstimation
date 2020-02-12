/* 
 * File:   MatrixTest.h
 * Author: Kodiak
 *
 * Created on March 4, 2019, 10:29 AM
 */

#ifndef MATRIXTEST_H
#define	MATRIXTEST_H

#include <Arduino.h>
#include "Matrix.h"

// calls all functions in the MatrixTest library
void MatrixTests(void);

/** TESTS
 * MatrixMultiply performs a matrix-matrix multiplication operation on two 3x3
 * matrices and returns the result in the third argument.
 */
void MatrixMultiplyTest();

/** TESTS
 * MatrixAdd performs a matrix addition operation on two 3x3 matrices and 
 * returns the result in the third argument.
 */
void MatrixAddTest();

/** TESTS
 * MatrixEquals checks if the two matrix arguments are equal. The result is
 * 0 if FALSE and 1 if TRUE to follow the standard C conventions of TRUE and
 * FALSE.
 */
void MatrixEqualsTest();

/******************************************************************************
 * Matrix - Scalar Operations
 *****************************************************************************/

/** TESTS
 * MatrixScalarMultiply performs the multiplication of a matrix by a scalar.
 * The result is returned in the third argument.
 */
void MatrixScalarMultiplyTest();

/** TESTS
 * MatrixScalarAdd performs the addition of a matrix by a scalar. The result
 * is returned in the third argument.
 */
void MatrixScalarAddTest();

/******************************************************************************
 * Unary Matrix Operations
 *****************************************************************************/

/** TESTS
 * MatrixDeterminant calculates the determinant of a matrix and returns the
 * value as a float.
 */
void MatrixDeterminantTest();

/** TESTS
 * MatrixTranspose calculates the transpose of a matrix and returns the
 * result through the second argument
 */
void MatrixTransposeTest();

/** TESTS
 * MatrixInverse calculates the inverse of a matrix and returns the
 * result through the second argument.
 */
void MatrixInverseTest();

/** TESTS
 * MatrixPrint sends a 3x3 array to standard output with clean formatting.
 * The formatting does not need to look like the expected output given to you
 * in MatricMath.c but each element of the matrix must be separated and have
 * distinguishable position (more than a single line of output).
 */
void MatrixPrintTest();

void MatrixCopyTest();

#endif	/* MATRIXTEST_H */

