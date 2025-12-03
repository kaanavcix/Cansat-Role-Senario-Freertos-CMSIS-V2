/*
 * servo.c
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */


#include "servo.h"



extern TIM_HandleTypeDef	htim2;
extern TIM_HandleTypeDef	htim3;


static uint16_t AngleToCCR(uint8_t angle) {
    if (angle > 180) angle = 180;
    return 500 + ((uint32_t)angle * 2000) / 180; // 0.5ms–2.5ms
}


void ServoInit()
{



	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}


void Servo1SetAngle(uint8_t angle)
{

	TIM2->CCR2 = AngleToCCR(angle);

}


void Servo2SetAngle(uint8_t angle)
{

	TIM3->CCR3 = AngleToCCR(angle);

}


