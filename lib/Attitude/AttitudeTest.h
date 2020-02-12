/* 
 * File:   AttitudeTest.h
 * Author: Kodiak
 *
 * Created on March 9, 2019, 2:13 PM
 */

#ifndef ATTITUDETEST_H
#define	ATTITUDETEST_H

#include <Arduino.h>
#include "Attitude.h"

void AttitudeTests();
void AttitudeDcmToEulerTest();
void AttitudeEulerToDcmTest();
void AttitudeRexpTest();

#endif	/* ATTITUDETEST_H */

