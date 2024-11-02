/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "constants.h"
#include "motor_message.h"
#include "math_ops.h"
#include "usart.h"
#include "robot.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg, CAN_HandleTypeDef *CAN_H);
void can_tx_init(CANTxMessage *msg, int ID);
void pack_cmd(CANTxMessage *msg, joint_control joint);
void unpack_reply(CANRxMessage msg, Robot_Struct *robot);
void EnterMotorMode(CANTxMessage *msg);
void ExitMotorMode(CANTxMessage *msg);
void Zero(CANTxMessage *msg);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

