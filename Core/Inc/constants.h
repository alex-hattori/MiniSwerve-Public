/*
 * constants.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define CAN_H1			hcan1
#define CAN_H2			hcan2

//CAN
// Master CAN ID ///
#define CAN_ID 0x0

#define BR_P_ID 2
#define FL_P_ID 6
#define FR_P_ID 4
#define BL_P_ID 8

#define BR_W_ID 1
#define FL_W_ID 5
#define FR_W_ID 3
#define BL_W_ID 7

/// Value Limits ///
#define P_MIN -500.0f
#define P_MAX 500.0f
#define V_MIN -200.0f
#define V_MAX 200.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

// Loop Frequencies
#define LOOP_FREQ 300.0f
#define IMU_FREQ 100.0f
#define PRINT_FREQ 50.0f

//SBUS
#define SBUS_BUF_LEN 50
#define SBUS_NUM_CH 18

//IMU
#define BNO055_ADDRESS 0x28

//RC PWM
#define PWM_MIN 1000
#define PWM_ZERO 1500
#define PWM_MAX 2000
#define PC6_PWM TIM_CHANNEL_1
#define PC7_PWM TIM_CHANNEL_2
#define PC8_PWM TIM_CHANNEL_3
#define PC9_PWM TIM_CHANNEL_4

#endif /* INC_CONSTANTS_H_ */
