/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : telecommand.c
  * @brief          : STM32F446 Multispectral Mechanical Filtering System
  *                   Complete implementation for dual 28BYJ-48 stepper motor control
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

#include "telecommand.h"
#include <string.h>
#include <stdlib.h>

MotorState_t motor1_state = {0, 0};
MotorState_t motor2_state = {0, 0};

static const uint8_t step_sequence[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

void Telecommand_Init(void)
{
    motor1_state.current_position = HOME_POSITION;
    motor1_state.current_step = 0;
    motor2_state.current_position = HOME_POSITION;
    motor2_state.current_step = 0;

    Telecommand_SetMotorPins(MOTOR_1, 0, 0, 0, 0);
    Telecommand_SetMotorPins(MOTOR_2, 0, 0, 0, 0);
}

void Telecommand_SetMotorPins(uint8_t motor_id, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
{
    if (motor_id == MOTOR_1) {
        HAL_GPIO_WritePin(STEP1_IN1_GPIO_Port, STEP1_IN1_Pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP1_IN2_GPIO_Port, STEP1_IN2_Pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP1_IN3_GPIO_Port, STEP1_IN3_Pin, in3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP1_IN4_GPIO_Port, STEP1_IN4_Pin, in4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (motor_id == MOTOR_2) {
        HAL_GPIO_WritePin(STEP2_IN1_GPIO_Port, STEP2_IN1_Pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP2_IN2_GPIO_Port, STEP2_IN2_Pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP2_IN3_GPIO_Port, STEP2_IN3_Pin, in3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(STEP2_IN4_GPIO_Port, STEP2_IN4_Pin, in4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void Telecommand_StepMotor(uint8_t motor_id, uint8_t step_index)
{
    uint8_t sequence_index = step_index % 8;
    
    if (motor_id == MOTOR_1) {
        // Motor 1: Normal sequence
        Telecommand_SetMotorPins(motor_id,
                               step_sequence[sequence_index][0],
                               step_sequence[sequence_index][1],
                               step_sequence[sequence_index][2],
                               step_sequence[sequence_index][3]);
    } else if (motor_id == MOTOR_2) {
        
#if MOTOR_2_REVERSE_SEQUENCE
        // Motor 2: Reverse step sequence to match Motor 1 direction
        uint8_t reverse_index = (7 - sequence_index);
        uint8_t final_index = reverse_index;
#else
        // Motor 2: Same step sequence as Motor 1
        uint8_t final_index = sequence_index;
#endif

#if MOTOR_2_REVERSE_PINS
        // Reverse pin order: IN4,IN3,IN2,IN1 instead of IN1,IN2,IN3,IN4
        Telecommand_SetMotorPins(motor_id,
                               step_sequence[final_index][3],
                               step_sequence[final_index][2],
                               step_sequence[final_index][1],
                               step_sequence[final_index][0]);
#else
        // Normal pin order
        Telecommand_SetMotorPins(motor_id,
                               step_sequence[final_index][0],
                               step_sequence[final_index][1],
                               step_sequence[final_index][2],
                               step_sequence[final_index][3]);
#endif
    }
}

uint16_t Telecommand_CalculateStepsToPosition(uint8_t current_pos, uint8_t target_pos)
{
    // Simple approach: always move forward to target position
    // This ensures consistent and predictable movement
    
    int16_t steps_forward;
    
    if (target_pos >= current_pos) {
        // Direct forward movement
        steps_forward = target_pos - current_pos;
    } else {
        // Wrap around: go forward through 0
        steps_forward = (4 - current_pos) + target_pos;
    }
    
    return (uint16_t)(steps_forward * STEPS_PER_90_DEGREES);
}

void Telecommand_MoveToPosition(uint8_t motor_id, uint8_t position)
{
    if (position > 3) return;

    MotorState_t* motor_state = (motor_id == MOTOR_1) ? &motor1_state : &motor2_state;

    uint16_t steps_needed = Telecommand_CalculateStepsToPosition(motor_state->current_position, position);

    for (uint16_t i = 0; i < steps_needed; i++) {
        Telecommand_StepMotor(motor_id, motor_state->current_step);
        motor_state->current_step++;
        HAL_Delay(STEP_DELAY_MS);
    }

    motor_state->current_position = position;
    Telecommand_SetMotorPins(motor_id, 0, 0, 0, 0);
}

void Telecommand_MoveBothMotors(uint8_t pos1, uint8_t pos2)
{
    if (pos1 > 3 || pos2 > 3) return;

    uint16_t steps1 = Telecommand_CalculateStepsToPosition(motor1_state.current_position, pos1);
    uint16_t steps2 = Telecommand_CalculateStepsToPosition(motor2_state.current_position, pos2);

    uint16_t max_steps = (steps1 > steps2) ? steps1 : steps2;

    for (uint16_t i = 0; i < max_steps; i++) {
        if (i < steps1) {
            Telecommand_StepMotor(MOTOR_1, motor1_state.current_step);
            motor1_state.current_step++;
        }

        if (i < steps2) {
            Telecommand_StepMotor(MOTOR_2, motor2_state.current_step);
            motor2_state.current_step++;
        }

        HAL_Delay(STEP_DELAY_MS);
    }

    motor1_state.current_position = pos1;
    motor2_state.current_position = pos2;

    Telecommand_SetMotorPins(MOTOR_1, 0, 0, 0, 0);
    Telecommand_SetMotorPins(MOTOR_2, 0, 0, 0, 0);
}

void Telecommand_ResetMotorStates(void)
{
    // Reset motor state tracking - use when motors are manually moved to home
    motor1_state.current_position = HOME_POSITION;
    motor1_state.current_step = 0;
    motor2_state.current_position = HOME_POSITION;
    motor2_state.current_step = 0;
}

void Telecommand_ReturnHome(void)
{
    Telecommand_MoveBothMotors(HOME_POSITION, HOME_POSITION);
}

FilterPosition_t Telecommand_GetFilterPosition(char filter_code)
{
    FilterPosition_t position = {0, 0};

    switch (filter_code) {
        case FILTER_M:  // Red + Red → Maroon Red
            position.disk1_position = POSITION_RED;
            position.disk2_position = POSITION_RED;
            break;

        case FILTER_F:  // Green + Green → Forest/Dark Green
            position.disk1_position = POSITION_GREEN;
            position.disk2_position = POSITION_GREEN;
            break;

        case FILTER_N:  // Blue + Blue → Navy Blue
            position.disk1_position = POSITION_BLUE;
            position.disk2_position = POSITION_BLUE;
            break;

        case FILTER_R:  // Red + Normal → Light Red
            position.disk1_position = POSITION_RED;
            position.disk2_position = POSITION_NORMAL;
            break;

        case FILTER_G:  // Green + Normal → Light Green
            position.disk1_position = POSITION_GREEN;
            position.disk2_position = POSITION_NORMAL;
            break;

        case FILTER_B:  // Blue + Normal → Light Blue
            position.disk1_position = POSITION_BLUE;
            position.disk2_position = POSITION_NORMAL;
            break;

        case FILTER_P:  // Red + Blue → Purple, Pink
            position.disk1_position = POSITION_RED;
            position.disk2_position = POSITION_BLUE;
            break;

        case FILTER_Y:  // Red + Green → Yellow, Brown
            position.disk1_position = POSITION_RED;
            position.disk2_position = POSITION_GREEN;
            break;

        case FILTER_C:  // Blue + Green → Cyan, Turquoise
            position.disk1_position = POSITION_BLUE;
            position.disk2_position = POSITION_GREEN;
            break;

        default:
            position.disk1_position = POSITION_NORMAL;
            position.disk2_position = POSITION_NORMAL;
            break;
    }

    return position;
}

void Telecommand_Execute(char* cmd_string)
{
    if (cmd_string == NULL) {
        return;
    }
    
    uint8_t cmd_len = strlen(cmd_string);
    
   if (cmd_len == 4) {
        // Full 4-character command: "XYZW" = X seconds of Y filter, then Z seconds of W filter
        uint8_t time1 = cmd_string[0] - '0';
        char filter1 = cmd_string[1];
        uint8_t time2 = cmd_string[2] - '0';
        char filter2 = cmd_string[3];

        if (time1 < 0 || time1 > 9 || time2 < 0 || time2 > 9) {
            return;
        }

        FilterPosition_t pos1 = Telecommand_GetFilterPosition(filter1);
        Telecommand_MoveBothMotors(pos1.disk1_position, pos1.disk2_position);
        HAL_Delay(time1 * 1000);

        FilterPosition_t pos2 = Telecommand_GetFilterPosition(filter2);
        Telecommand_MoveBothMotors(pos2.disk1_position, pos2.disk2_position);
        HAL_Delay(time2 * 1000);
        Telecommand_ReturnHome();
    }
    // If command length is not 2 or 4, ignore it
}
