/* 
 * File:   Matrix.h
 * Author: Kodiak
 *
 * Created on March 3, 2019, 7:16 PM
 */

#ifndef MATRIX_H
#define	MATRIX_H

/**
 * FP_DELTA defines the tolerance for testing equality for floating-point numbers. Used within
 * MatrixEquals() and is useful for comparison of return values of MatrixDeterminant() and
 * MatrixTrace().
 */
#define FP_DELTA 0.0001

#include <Arduino.h>
#include <stdio.h>
#include <Math.h>

/**
 * MatrixMultiply performs a matrix-matrix multiplication operation on two 3x3
 * matrices and returns the result in the third argument.
 */
void MatrixMultiply(float m1[3][3], float m2[3][3], float res[3][3]);

/**
 * MatrixAdd performs a matrix addition operation on two 3x3 matrices and 
 * returns the result in the third argument.
 */
void MatrixAdd(float m1[3][3], float m2[3][3], float res[3][3]);

/**
 * multiplies matrix m by vector v and stores in res
 * @param m
 * @param v
 * @param res
 */
void MatrixVectorMultiply(float m[3][3], float v[3], float res[3]);

/**
 * adds vector v1 to vector v2 and stores in res
 * @param v1
 * @param v2
 * @param res
 */
void MatrixVectorAdd(float v1[3], float v2[3], float res[3]);

/**
 * subtracts vector v2 from vector v1 and stores in res
 * @param v1
 * @param v2
 * @param res
 */
void MatrixVectorSub(float v1[3], float v2[3], float res[3]);

/**
 * returns the norm of vector v 
 */
float MatrixVectorNorm(float v[3]);

/**
 * MatrixEquals checks if the two matrix arguments are equal. The result is
 * 0 if FALSE and 1 if TRUE to follow the standard C conventions of TRUE and
 * FALSE.
 */
int MatrixEquals(float m1[3][3], float m2[3][3]);

/******************************************************************************
 * Matrix - Scalar Operations
 *****************************************************************************/

/**
 * MatrixScalarMultiply performs the multiplication of a matrix by a scalar.
 * The result is returned in the third argument.
 */
void MatrixScalarMultiply(float x, float m[3][3], float res[3][3]);

/**
 * MatrixScalarAdd performs the addition of a matrix by a scalar. The result
 * is returned in the third argument.
 */
void MatrixScalarAdd(float x, float m[3][3], float res[3][3]);

/******************************************************************************
 * Unary Matrix Operations
 *****************************************************************************/

/**
 * MatrixDeterminant calculates the determinant of a matrix and returns the
 * value as a float.
 */
float MatrixDeterminant(float m[3][3]);

/**
 * MatrixTranspose calculates the transpose of a matrix and returns the
 * result through the second argument
 */
void MatrixTranspose(float m[3][3], float res[3][3]);

/**
 * MatrixInverse calculates the inverse of a matrix and returns the
 * result through the second argument.
 */
void MatrixInverse(float m[3][3], float res[3][3]);

/**
 * MatrixPrint sends a 3x3 array to standard output with clean formatting.
 */
void MatrixPrint(float m[3][3]);

void MatrixCopy(float m1[3][3], float m2[3][3]);

#endif	/* MATRIX_H */