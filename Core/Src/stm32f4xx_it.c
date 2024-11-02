/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "constants.h"
#include "structs.h"
#include "sbus.h"
#include "robot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim3;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
		int startIndex = SBUS_BUF_LEN+1;
		for(int i = 0; i<SBUS_BUF_LEN/2; i++){
			if(sbus2_rxbuffer[i] == 0x0F){
				startIndex = i;
				break;
				}
			}
		if(startIndex != (SBUS_BUF_LEN+1) && sbus2_rxbuffer[startIndex+24]==0x00){
			robot.radio2_rx.rx_chan[0]  = ((sbus2_rxbuffer[startIndex+1]    |sbus2_rxbuffer[startIndex+2]<<8)                 & 0x07FF);
			robot.radio2_rx.rx_chan[1]  = ((sbus2_rxbuffer[startIndex+2]>>3 |sbus2_rxbuffer[startIndex+3]<<5)                 & 0x07FF);
			robot.radio2_rx.rx_chan[2]  = ((sbus2_rxbuffer[startIndex+3]>>6 |sbus2_rxbuffer[startIndex+4]<<2 |sbus2_rxbuffer[startIndex+5]<<10)  & 0x07FF);
			robot.radio2_rx.rx_chan[3]  = ((sbus2_rxbuffer[startIndex+5]>>1 |sbus2_rxbuffer[startIndex+6]<<7)                 & 0x07FF);
			robot.radio2_rx.rx_chan[4]  = ((sbus2_rxbuffer[startIndex+6]>>4 |sbus2_rxbuffer[startIndex+7]<<4)                 & 0x07FF);
			robot.radio2_rx.rx_chan[5]  = ((sbus2_rxbuffer[startIndex+7]>>7 |sbus2_rxbuffer[startIndex+8]<<1 |sbus2_rxbuffer[startIndex+9]<<9)   & 0x07FF);
			robot.radio2_rx.rx_chan[6]  = ((sbus2_rxbuffer[startIndex+9]>>2 |sbus2_rxbuffer[startIndex+10]<<6)                & 0x07FF);
			robot.radio2_rx.rx_chan[7]  = ((sbus2_rxbuffer[startIndex+10]>>5|sbus2_rxbuffer[startIndex+11]<<3)                & 0x07FF);
			robot.radio2_rx.rx_chan[8]  = ((sbus2_rxbuffer[startIndex+12]   |sbus2_rxbuffer[startIndex+13]<<8)                & 0x07FF);
			robot.radio2_rx.rx_chan[9]  = ((sbus2_rxbuffer[startIndex+13]>>3|sbus2_rxbuffer[startIndex+14]<<5)                & 0x07FF);
			robot.radio2_rx.rx_chan[10] = ((sbus2_rxbuffer[startIndex+14]>>6|sbus2_rxbuffer[startIndex+15]<<2|sbus2_rxbuffer[16]<<10) & 0x07FF);
			robot.radio2_rx.rx_chan[11] = ((sbus2_rxbuffer[startIndex+16]>>1|sbus2_rxbuffer[startIndex+17]<<7)                & 0x07FF);
			robot.radio2_rx.rx_chan[12] = ((sbus2_rxbuffer[startIndex+17]>>4|sbus2_rxbuffer[startIndex+18]<<4)                & 0x07FF);
			robot.radio2_rx.rx_chan[13] = ((sbus2_rxbuffer[startIndex+18]>>7|sbus2_rxbuffer[startIndex+19]<<1|sbus2_rxbuffer[startIndex+20]<<9)  & 0x07FF);
			robot.radio2_rx.rx_chan[14] = ((sbus2_rxbuffer[startIndex+20]>>2|sbus2_rxbuffer[startIndex+21]<<6)                & 0x07FF);
			robot.radio2_rx.rx_chan[15] = ((sbus2_rxbuffer[startIndex+21]>>5|sbus2_rxbuffer[startIndex+22]<<3)                & 0x07FF);
			robot.radio2_rx.rx_chan[16] = ((sbus2_rxbuffer[startIndex+23])      & 0x0001) ? 2047 : 0;
			robot.radio2_rx.rx_chan[17] = ((sbus2_rxbuffer[startIndex+23] >> 1) & 0x0001) ? 2047 : 0;
		}
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
  HAL_UART_Receive_DMA(&huart3, sbus2_rxbuffer, SBUS_BUF_LEN);

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET ); //For timing (Indicator 2)
    sbus_update(&robot.radio_rx);
    sbus_update(&robot.radio2_rx);
	for(int i = 0; i<8; i++){
		int no_mesage = HAL_CAN_GetRxMessage(&CAN_H1, CAN_RX_FIFO0, &can1_rx.rx_header, can1_rx.data);
		if(!no_mesage){
			unpack_reply(can1_rx, &robot);
		}
		int no_mesage2 = HAL_CAN_GetRxMessage(&CAN_H2, CAN_RX_FIFO0, &can2_rx.rx_header, can2_rx.data);
		if(!no_mesage2){
			unpack_reply(can2_rx, &robot);
		}
	}
	robot.imu_update_counter++;
	robot.print_counter++;
	if(robot.imu_update_counter>(LOOP_FREQ/IMU_FREQ)){
		robot.imu_update_counter = 0;
		robot.sendCounter = 1;
	}
	control(&robot);
	if(robot.print_counter>(LOOP_FREQ/PRINT_FREQ)){
		debug_print(&robot);
		robot.print_counter = 0;
	}
	PackAll();
	if(robot.radio_rx.is_enabled && !robot.is_enabled){
		EnterMotorMode(&FL_P_MSG);
		EnterMotorMode(&FL_W_MSG);
		EnterMotorMode(&FR_P_MSG);
		EnterMotorMode(&FR_W_MSG);
		EnterMotorMode(&BL_P_MSG);
		EnterMotorMode(&BL_W_MSG);
		EnterMotorMode(&BR_P_MSG);
		EnterMotorMode(&BR_W_MSG);
		robot.yawGoal = robot.yaw; //In case of closed loop control
		robot.is_enabled = 1;
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET ); // LED 1
	    printf("Enabled\r\n");
	}
	if(!robot.radio_rx.is_enabled){
		ExitMotorMode(&FL_P_MSG);
		ExitMotorMode(&FL_W_MSG);
		ExitMotorMode(&FR_P_MSG);
		ExitMotorMode(&FR_W_MSG);
		ExitMotorMode(&BL_P_MSG);
		ExitMotorMode(&BL_W_MSG);
		ExitMotorMode(&BR_P_MSG);
		ExitMotorMode(&BR_W_MSG);
		robot.is_enabled = 0;
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET ); // LED 1
	}
	WriteAll();

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET ); //For timing (middle LED)
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);

	char c = Serial2RxBuffer[0];
	switch(c){
		case(27):
				break;
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	int startIndex = SBUS_BUF_LEN+1;
	for(int i = 0; i<SBUS_BUF_LEN/2; i++){
		if(sbus_rxbuffer[i] == 0x0F){
			startIndex = i;
			break;
			}
		}
	if(startIndex != (SBUS_BUF_LEN+1) && sbus_rxbuffer[startIndex+24]==0x00){
		robot.radio_rx.rx_chan[0]  = ((sbus_rxbuffer[startIndex+1]    |sbus_rxbuffer[startIndex+2]<<8)                 & 0x07FF);
		robot.radio_rx.rx_chan[1]  = ((sbus_rxbuffer[startIndex+2]>>3 |sbus_rxbuffer[startIndex+3]<<5)                 & 0x07FF);
		robot.radio_rx.rx_chan[2]  = ((sbus_rxbuffer[startIndex+3]>>6 |sbus_rxbuffer[startIndex+4]<<2 |sbus_rxbuffer[startIndex+5]<<10)  & 0x07FF);
		robot.radio_rx.rx_chan[3]  = ((sbus_rxbuffer[startIndex+5]>>1 |sbus_rxbuffer[startIndex+6]<<7)                 & 0x07FF);
		robot.radio_rx.rx_chan[4]  = ((sbus_rxbuffer[startIndex+6]>>4 |sbus_rxbuffer[startIndex+7]<<4)                 & 0x07FF);
		robot.radio_rx.rx_chan[5]  = ((sbus_rxbuffer[startIndex+7]>>7 |sbus_rxbuffer[startIndex+8]<<1 |sbus_rxbuffer[startIndex+9]<<9)   & 0x07FF);
		robot.radio_rx.rx_chan[6]  = ((sbus_rxbuffer[startIndex+9]>>2 |sbus_rxbuffer[startIndex+10]<<6)                & 0x07FF);
		robot.radio_rx.rx_chan[7]  = ((sbus_rxbuffer[startIndex+10]>>5|sbus_rxbuffer[startIndex+11]<<3)                & 0x07FF);
		robot.radio_rx.rx_chan[8]  = ((sbus_rxbuffer[startIndex+12]   |sbus_rxbuffer[startIndex+13]<<8)                & 0x07FF);
		robot.radio_rx.rx_chan[9]  = ((sbus_rxbuffer[startIndex+13]>>3|sbus_rxbuffer[startIndex+14]<<5)                & 0x07FF);
		robot.radio_rx.rx_chan[10] = ((sbus_rxbuffer[startIndex+14]>>6|sbus_rxbuffer[startIndex+15]<<2|sbus_rxbuffer[16]<<10) & 0x07FF);
		robot.radio_rx.rx_chan[11] = ((sbus_rxbuffer[startIndex+16]>>1|sbus_rxbuffer[startIndex+17]<<7)                & 0x07FF);
		robot.radio_rx.rx_chan[12] = ((sbus_rxbuffer[startIndex+17]>>4|sbus_rxbuffer[startIndex+18]<<4)                & 0x07FF);
		robot.radio_rx.rx_chan[13] = ((sbus_rxbuffer[startIndex+18]>>7|sbus_rxbuffer[startIndex+19]<<1|sbus_rxbuffer[startIndex+20]<<9)  & 0x07FF);
		robot.radio_rx.rx_chan[14] = ((sbus_rxbuffer[startIndex+20]>>2|sbus_rxbuffer[startIndex+21]<<6)                & 0x07FF);
		robot.radio_rx.rx_chan[15] = ((sbus_rxbuffer[startIndex+21]>>5|sbus_rxbuffer[startIndex+22]<<3)                & 0x07FF);
		robot.radio_rx.rx_chan[16] = ((sbus_rxbuffer[startIndex+23])      & 0x0001) ? 2047 : 0;
		robot.radio_rx.rx_chan[17] = ((sbus_rxbuffer[startIndex+23] >> 1) & 0x0001) ? 2047 : 0;
	}
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim8_ch1);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
  HAL_UART_Receive_DMA(&huart1, sbus_rxbuffer, SBUS_BUF_LEN);

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void PackAll()
{
	pack_cmd(&FL_P_MSG, robot.FL_P_CTRL);
	pack_cmd(&FR_P_MSG, robot.FR_P_CTRL);
	pack_cmd(&BL_P_MSG, robot.BL_P_CTRL);
	pack_cmd(&BR_P_MSG, robot.BR_P_CTRL);
	pack_cmd(&FL_W_MSG, robot.FL_W_CTRL);
	pack_cmd(&FR_W_MSG, robot.FR_W_CTRL);
	pack_cmd(&BL_W_MSG, robot.BL_W_CTRL);
	pack_cmd(&BR_W_MSG, robot.BR_W_CTRL);

	int fl_w_pwm = float_to_rcpwm(robot.FL_W_CTRL.v_des, V_MIN, V_MAX, robot.FL_W_PWM.min, robot.FL_W_PWM.max);
	int fr_w_pwm = float_to_rcpwm(robot.FR_W_CTRL.v_des, V_MIN, V_MAX, robot.FR_W_PWM.min, robot.FR_W_PWM.max);
	int bl_w_pwm = float_to_rcpwm(robot.BL_W_CTRL.v_des, V_MIN, V_MAX, robot.BL_W_PWM.min, robot.BL_W_PWM.max);
	int br_w_pwm = float_to_rcpwm(robot.BR_W_CTRL.v_des, V_MIN, V_MAX, robot.BR_W_PWM.min, robot.BR_W_PWM.max);
	__HAL_TIM_SET_COMPARE(&htim3, PC6_PWM, fl_w_pwm);
	__HAL_TIM_SET_COMPARE(&htim3, PC7_PWM, fr_w_pwm);
	__HAL_TIM_SET_COMPARE(&htim3, PC8_PWM, bl_w_pwm);
	__HAL_TIM_SET_COMPARE(&htim3, PC9_PWM, br_w_pwm);
}
void WriteAll()
{
	// CAN only put 3 messages into mailbox at a time, hence the split up.
	// Must have at least one device on each bus or code will hang because no acks will be received
	while(HAL_CAN_IsTxMessagePending(&CAN_H2,TxMailbox2)!=0){
		int no_mesage2 = HAL_CAN_GetRxMessage(&CAN_H2, CAN_RX_FIFO0, &can2_rx.rx_header, can2_rx.data);
		if(!no_mesage2){
			unpack_reply(can2_rx, &robot);
		}
	}
	while(HAL_CAN_IsTxMessagePending(&CAN_H1,TxMailbox1)!=0){
		int no_mesage = HAL_CAN_GetRxMessage(&CAN_H1, CAN_RX_FIFO0, &can1_rx.rx_header, can1_rx.data);
		if(!no_mesage){
			unpack_reply(can1_rx, &robot);
		}
	}
	HAL_CAN_AddTxMessage(&CAN_H1, &BR_P_MSG.tx_header, BR_P_MSG.data, &TxMailbox1);
	HAL_CAN_AddTxMessage(&CAN_H2, &FL_P_MSG.tx_header, FL_P_MSG.data, &TxMailbox2);
	HAL_CAN_AddTxMessage(&CAN_H1, &FR_P_MSG.tx_header, FR_P_MSG.data, &TxMailbox1);
	HAL_CAN_AddTxMessage(&CAN_H2, &BL_P_MSG.tx_header, BL_P_MSG.data, &TxMailbox2);
	while(HAL_CAN_IsTxMessagePending(&CAN_H2,TxMailbox2)!=0){
		int no_mesage2 = HAL_CAN_GetRxMessage(&CAN_H2, CAN_RX_FIFO0, &can2_rx.rx_header, can2_rx.data);
		if(!no_mesage2){
			unpack_reply(can2_rx, &robot);
		}
	}
	while(HAL_CAN_IsTxMessagePending(&CAN_H1,TxMailbox1)!=0){
		int no_mesage = HAL_CAN_GetRxMessage(&CAN_H1, CAN_RX_FIFO0, &can1_rx.rx_header, can1_rx.data);
		if(!no_mesage){
			unpack_reply(can1_rx, &robot);
		}
	}
	HAL_CAN_AddTxMessage(&CAN_H1, &BR_W_MSG.tx_header, BR_W_MSG.data, &TxMailbox1);
	HAL_CAN_AddTxMessage(&CAN_H2, &FL_W_MSG.tx_header, FL_W_MSG.data, &TxMailbox2);
	HAL_CAN_AddTxMessage(&CAN_H1, &FR_W_MSG.tx_header, FR_W_MSG.data, &TxMailbox1);
	HAL_CAN_AddTxMessage(&CAN_H2, &BL_W_MSG.tx_header, BL_W_MSG.data, &TxMailbox2);
	while(HAL_CAN_IsTxMessagePending(&CAN_H2,TxMailbox2)!=0){
		int no_mesage2 = HAL_CAN_GetRxMessage(&CAN_H2, CAN_RX_FIFO0, &can2_rx.rx_header, can2_rx.data);
		if(!no_mesage2){
			unpack_reply(can2_rx, &robot);
		}
	}
	while(HAL_CAN_IsTxMessagePending(&CAN_H1,TxMailbox1)!=0){
		int no_mesage = HAL_CAN_GetRxMessage(&CAN_H1, CAN_RX_FIFO0, &can1_rx.rx_header, can1_rx.data);
		if(!no_mesage){
			unpack_reply(can1_rx, &robot);
		}
	}
}


/* USER CODE END 1 */
