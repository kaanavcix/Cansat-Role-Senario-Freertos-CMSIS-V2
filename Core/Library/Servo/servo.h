/*
 * servo.h
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */

#ifndef LIBRARY_SERVO_SERVO_H_
#define LIBRARY_SERVO_SERVO_H_


#include "../Utility/utiltity.h"

#define ANGLE_LOW 0
#define ANGLE_HIGH 180;




void ServoInit();

void Servo1SetAngle(uint8_t angle);
void Servo2SetAngle(uint8_t angle);


#endif /* LIBRARY_SERVO_SERVO_H_ */
