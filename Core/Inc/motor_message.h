/*
 * leg_message.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Alex
 */

#ifndef INC_MOTOR_MESSAGE_H_
#define INC_MOTOR_MESSAGE_H_

typedef struct {
    float p_des, v_des, kp, kd, t_ff;
}joint_control;

typedef struct {
    float p, v, t;
}joint_state;

typedef struct {
    float mult, angle, errUsed;
}pivot_cmd;

typedef struct {
	int microseconds, min, max;
}pwm_cmd;

#endif /* INC_MOTOR_MESSAGE_H_ */
