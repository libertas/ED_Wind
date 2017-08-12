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
#include "resistor.h"
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

float motor_dest_heights[4] = {0};
float motor_heights[4] = {0};
bool motor_resetting = true;
bool motion_resetting = true;

uint16_t get_pos_x()
{
	return pos_x;
}

uint16_t get_pos_y()
{
	return pos_y;
}

void motion_init_pid()
{
	pid_config(&px);
	pid_config(&py);

	px.kp = 4.0;
	px.ki = 0.0;
	px.kd = 5.0;

	py.kp = 4.0;
	py.ki = 0.0;
	py.kd = 5.0;

	for(int i = 0; i < 4; i++) {
		pid_config(&(motor_pids[i]));

		motor_pids[i].kp = 0.01;
		motor_pids[i].ki = 0.0;
		motor_pids[i].kd = 0.01;
	}
}

void motion_init(TIM_HandleTypeDef *htim)
{
	holes[0][0] = 180; holes[0][1] = 84;
	holes[1][0] = 318; holes[1][1] = 86;
	holes[2][0] = 466; holes[2][1] = 86;

	holes[3][0] = 180; holes[3][1] = 232;
	holes[4][0] = 320; holes[4][1] = 236;
	holes[5][0] = 464; holes[5][1] = 232;

	holes[6][0] = 186; holes[6][1] = 370;
	holes[7][0] = 316; holes[7][1] = 372;
	holes[8][0] = 460; holes[8][1] = 376;

	motion_htim = htim;

	motion_init_pid();

	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_4);

	motor_reset();
	motion_reset();
}

void motor_control()
{
	if(motor_resetting) {
		return;
	}

	float v[4];

	for(int i = 0; i < 4; i++) {
		motor_heights[i] = resistor_get(i);
		switch(i) {
		case 0:
		case 1:
			motor_heights[i] *= 1.5f;
			break;
		default:
			break;
		}

		motor_pids[i].actual_value = motor_heights[i];
		motor_pids[i].set_value = motor_dest_heights[i];
		v[i] = pid_realize(&(motor_pids[i]));

		// !!
		if(fabsf(motor_pids[i].error) > 15.0f) {
			v[i] = fabsf(v[i]) / v[i];
		} else {
			v[i] = 0;
		}

		if(motor_heights[i] > 3500 && v[i] > 0) {
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
	motion_resetting = false;
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

	for(int i = 0; i < 4; i++) {
		motor_heights[i] = 0;
	}
}

void motor_move(float heights[4])
{
	for(int i = 0; i < 4; i++) {
		motor_dest_heights[i] = heights[i];
	}
}

void motor_move_mid()
{
	float mps[4];

	for(int i = 0; i < 4; i++) {
		mps[i] = MOTOR_MID;
	}

	motor_move(mps);
}

void motion_reset()
{
	motor_start();

	motion_resetting = true;
	pos_x = 0;
	pos_y = 0;
}

void motion_control()
{
	float x, y;

	if(motion_resetting) {
		x = 0;
		y = 0;
	} else {
		extern char currentTask;

		if(fabsf(px.error) < 40 && fabsf(py.error) < 40) {
			switch(currentTask) {
			case '4':
				px.ki = 0.03;
				py.ki = 0.03;
				break;
			case '3':
				px.ki = 0.06;
				py.ki = 0.06;
				break;
			case '2':
			default:
				px.ki = 0.02;
				py.ki = 0.02;
				break;
			}

		} else if(fabsf(px.error) < 75 && fabsf(py.error) < 75) {
			switch(currentTask) {
			case '4':
				px.ki = 0.03;
				py.ki = 0.03;
				break;
			case '3':
				px.ki = 0.04;
				py.ki = 0.04;
				break;
			case '2':
			default:
				px.ki = 0.02;
				py.ki = 0.02;
				break;
			}
		} else {
			px.ki = 0;
			py.ki = 0;
		}

		px.actual_value = pos_x;
		px.set_value = dest_x;
		x = pid_realize(&px);

		py.actual_value = pos_y;
		py.set_value = dest_y;
		y = pid_realize(&py);
	}


	float mps[4];
	mps[0] = -y + MOTOR_MID;
	mps[1] = y + MOTOR_MID;
	mps[2] = -x + MOTOR_MID;
	mps[3] = x + MOTOR_MID;

	motor_move(mps);
}

void move_to_pos(uint16_t x, uint16_t y)
{
	dest_x = x;
	dest_y = y;
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
