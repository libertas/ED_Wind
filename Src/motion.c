/*
 * motion.c
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#include <math.h>
#include <string.h>

#include "cmsis_os.h"

#include "motion.h"
#include "pid.h"
#include "pwm.h"
#include "time.h"

bool debugFlag = false;

TIM_HandleTypeDef *motion_htim;

mypid_t px, py;

uint16_t pos_x = 320;
uint16_t pos_y = 240;

uint16_t dest_x = 320;
uint16_t dest_y = 240;

uint16_t holes[9][2] = {0};
bool holes_available = false;

mypid_t motor_pids[4];
float motor_dest_angles[4];

bool motor_resetting = true;

void motion_init(TIM_HandleTypeDef *htim)
{
	motion_htim = htim;

	pid_config(&px);
	pid_config(&py);

	px.kp = 1.0;
	px.ki = 0.0;
	px.kd = 0.0;

	py.kp = 1.0;
	py.ki = 0.0;
	py.kd = 0.0;

	for(int i = 0; i < 4; i++) {
		motor_dest_angles[i] = 0.0f;

		pid_config(&(motor_pids[i]));

		motor_pids[i].kp = 1.0;
		motor_pids[i].ki = 0.0;
		motor_pids[i].kd = 0.0;
	}

	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_4);

	motor_reset();
}

void motor_control(struct kine_state *ks)
{
	if(motor_resetting) {
		return;
	}

	float x[4];
	x[0] = (ks->x);
	x[1] = -(ks->x);
	x[2] = -(ks->y);
	x[3] = (ks->y);


	float v[4];

	for(int i = 0; i < 4; i++) {
		motor_pids[i].actual_value = x[i];
		motor_pids[i].set_value = motor_dest_angles[i];
		v[i] = pid_realize(&(motor_pids[i]));

		// !!
		if(fabsf(v[i]) > 0.001f) {
			v[i] = fabsf(v[i]) / v[i];
		} else {
			v[i] = 0;
		}

		if(debugFlag) {
			sl_send(5, 0, &(v[i]), 4);
		}
	}

	l298n_set(TIM_CHANNEL_1, -v[0]);
	l298n_set(TIM_CHANNEL_2, -v[1]);
	l298n_set(TIM_CHANNEL_3, -v[2]);
	l298n_set(TIM_CHANNEL_4, -v[3]);
}

void motor_start()
{
	motor_resetting = false;
}

void motor_stop()
{
	motor_resetting = true;
}

void motor_reset()
{
	motor_resetting = true;

	l298n_set(TIM_CHANNEL_1, 1);
	l298n_set(TIM_CHANNEL_2, 1);
	l298n_set(TIM_CHANNEL_3, 1);
	l298n_set(TIM_CHANNEL_4, 1);

	osDelay(3000);

	l298n_set(TIM_CHANNEL_1, -1);
	l298n_set(TIM_CHANNEL_2, -1);
	l298n_set(TIM_CHANNEL_3, -1);
	l298n_set(TIM_CHANNEL_4, -1);

	osDelay(845);

	l298n_set(TIM_CHANNEL_1, 0);
	l298n_set(TIM_CHANNEL_2, 0);
	l298n_set(TIM_CHANNEL_3, 0);
	l298n_set(TIM_CHANNEL_4, 0);
}

void motor_move(float angles[4])
{
	for(int i = 0; i < 4; i++) {
		if(fabsf(angles[i]) > MOTOR_ANGLE_LIMIT) {
			angles[i] = angles[i] / fabsf(angles[i]) * MOTOR_ANGLE_LIMIT;
		}
		motor_dest_angles[i] = angles[i];
	}
}

void motion_control()
{
	float x, y;

	px.actual_value = pos_x;
	px.set_value = dest_x;
	x = pid_realize(&px);

	py.actual_value = pos_y;
	py.set_value = dest_y;
	y = pid_realize(&py);

	float mps[4];
	mps[0] = x;
	mps[1] = -x;
	mps[2] = -y;
	mps[3] = y;

	motor_move(mps);
}

void move_to_pos(uint16_t x, uint16_t y)
{
	dest_x = x;
	dest_y = y;
}

bool move_to_hole(uint8_t hole)
{
	if(!holes_available) {
		return false;
	}

	move_to_pos(holes[hole][0], holes[hole][1]);

	return true;
}

void l298n_set(uint32_t channel, float duty)
{
	const GPIO_TypeDef* port = GPIOE;
	uint16_t pin_a, pin_b;

	switch(channel) {
	case TIM_CHANNEL_1:
		pin_a = GPIO_PIN_8;
		pin_b = GPIO_PIN_9;
		break;
	case TIM_CHANNEL_2:
		pin_a = GPIO_PIN_10;
		pin_b = GPIO_PIN_11;
		break;
	case TIM_CHANNEL_3:
		pin_a = GPIO_PIN_12;
		pin_b = GPIO_PIN_13;
		break;
	case TIM_CHANNEL_4:
		pin_a = GPIO_PIN_14;
		pin_b = GPIO_PIN_15;
		break;
	default:
		return;
	}

	if(duty >= 0) {
		HAL_GPIO_WritePin(port, pin_a, 1);
		HAL_GPIO_WritePin(port, pin_b, 0);
	} else {
		HAL_GPIO_WritePin(port, pin_a, 0);
		HAL_GPIO_WritePin(port, pin_b, 1);
		duty = -duty;
	}

	if(duty > 1) {
		duty = 1;
	}

	set_duty(motion_htim, channel, duty);
}
