/*
* mpu6050.h
*
* Created on: Jul 23, 2025
* Author: kaana
*/

#ifndef LIBRARY_ACCELERATION_MPU6050_H_
#define LIBRARY_ACCELERATION_MPU6050_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>

// MPU6050 başlatma ve kalibrasyon fonksiyonları
void MPU6050_Init(void);
void MPU6050_Calibrate(void);

// Ham sensör okuma fonksiyonları
float MPU6050_Temperature(void);
float MPU6050_Read_Accel_X(void);
float MPU6050_Read_Accel_Y(void);
float MPU6050_Read_Accel_Z(void);
float MPU6050_Read_Gyro_X(void);
float MPU6050_Read_Gyro_Y(void);
float MPU6050_Read_Gyro_Z(void);

float MPU6050_Read_Pitch_Angle(void);
float MPU6050_Read_Roll_Angle(void);
float MPU6050_Read_Yaw_Angle(void);

// Kalman filtreli fonksiyonlar
float MPU6050_Kalman_Roll_Angle(void);
double MPU6050_Kalman_Accel_X(void);
double MPU6050_Kalman_Accel_Y(void);
double MPU6050_Kalman_Accel_Z(void);
double MPU6050_Kalman_Gyro_X(void);
double MPU6050_Kalman_Gyro_Y(void);
double MPU6050_Kalman_Gyro_Z(void);
float MPU6050_Kalman_Temp(void);

#endif /* LIBRARY_ACCELERATION_MPU6050_H_ */
