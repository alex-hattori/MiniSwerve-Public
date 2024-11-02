/*
 * structs.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "motor_message.h"
#include "can.h"
#include "constants.h"
#include "sbus.h"
#include "robot.h"
#include "LED.h"

extern CANRxMessage can1_rx, can2_rx; //CAN received messages
extern CANTxMessage FL_P_MSG,FL_W_MSG,FR_P_MSG,FR_W_MSG,BL_P_MSG,BL_W_MSG,BR_P_MSG,BR_W_MSG;

extern Robot_Struct robot;
extern LED_Struct leds;

extern uint32_t TxMailbox1, TxMailbox2;

#endif /* INC_STRUCTS_H_ */
