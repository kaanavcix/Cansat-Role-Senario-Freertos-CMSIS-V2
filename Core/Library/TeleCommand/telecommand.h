/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : telecommand.h
  * @brief          : Header for telecommand.c file.
  *                   STM32F446 Multispectral Mechanical Filtering System
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TELECOMMAND_H
#define __TELECOMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"

#define STEPS_PER_90_DEGREES 2048
#define HOME_POSITION 0
#define STEP_DELAY_MS 2

// Motor direction correction options
#define MOTOR_2_REVERSE_SEQUENCE 1  // Set to 1 to reverse Motor 2 direction
#define MOTOR_2_REVERSE_PINS 0      // Set to 1 to reverse Motor 2 pin order

typedef enum {
    FILTER_M = 'M',  // Red + Red → Maroon Red
    FILTER_F = 'F',  // Green + Green → Forest/Dark Green
    FILTER_N = 'N',  // Blue + Blue → Navy Blue
    FILTER_R = 'R',  // Red + Normal OR Normal + Red → Light Red
    FILTER_G = 'G',  // Green + Normal OR Normal + Green → Light Green
    FILTER_B = 'B',  // Blue + Normal OR Normal + Blue → Light Blue
    FILTER_P = 'P',  // Red + Blue OR Blue + Red → Purple, Pink
    FILTER_Y = 'Y',  // Red + Green OR Green + Red → Yellow, Brown
    FILTER_C = 'C'   // Blue + Green OR Green + Blue → Cyan, Turquoise
} FilterCode_t;

typedef struct {
    uint8_t disk1_position;  // 0=Normal, 1=Red, 2=Green, 3=Blue
    uint8_t disk2_position;  // 0=Normal, 1=Red, 2=Green, 3=Blue
} FilterPosition_t;

typedef enum {
    MOTOR_1 = 1,
    MOTOR_2 = 2
} MotorID_t;

typedef enum {
    POSITION_NORMAL = 0,  // Home position (0°)
    POSITION_BLUE = 1,    // 90° from home
    POSITION_GREEN = 2,   // 180° from home
    POSITION_RED = 3      // 270° from home
} FilterPositionEnum_t;

typedef struct {
    uint8_t current_position;
    uint16_t current_step;
} MotorState_t;

extern MotorState_t motor1_state;
extern MotorState_t motor2_state;

void Telecommand_Init(void);
void Telecommand_Execute(char* cmd_string);
void Telecommand_MoveToPosition(uint8_t motor_id, uint8_t position);
void Telecommand_MoveBothMotors(uint8_t pos1, uint8_t pos2);
void Telecommand_ReturnHome(void);
FilterPosition_t Telecommand_GetFilterPosition(char filter_code);
uint16_t Telecommand_CalculateStepsToPosition(uint8_t current_pos, uint8_t target_pos);
void Telecommand_StepMotor(uint8_t motor_id, uint8_t step_index);
void Telecommand_SetMotorPins(uint8_t motor_id, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4);
void Telecommand_ResetMotorStates(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELECOMMAND_H */
