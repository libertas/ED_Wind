/*
 * motion.c
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#include <math.h>

#include "motion.h"
#include "pid.h"
#include "pwm.h"

TIM_HandleTypeDef *motion_htim;

mypid_t px, py;

void motion_init(TIM_HandleTypeDef *htim)
{
	motion_htim = htim;

	pid_config(&px);
	pid_config(&py);

	px.kp = 2;
	px.ki = 0;
	px.kd = 0;

	py.kp = 2;
	py.ki = 0;
	py.kd = 0;

	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(motion_htim, TIM_CHANNEL_4);
}

void motion_control(float dest_x, float dest_y, struct kine_state *ks)
{
	float vx, vy;

	px.actual_value = ks->x;
	px.set_value = dest_x;
	vx = pid_realize(&px);

	py.actual_value = ks->y;
	px.set_value = dest_y;
	vy = pid_realize(&py);

	l298n_set(TIM_CHANNEL_1, vx);
	l298n_set(TIM_CHANNEL_2, vy);
	l298n_set(TIM_CHANNEL_3, -vx);
	l298n_set(TIM_CHANNEL_4, -vy);
}

void l298n_set(uint32_t channel, float duty)
{
	const GPIO_TypeDef* port = GPIOB;
	uint16_t pin_a, pin_b;

	switch(channel) {
	case TIM_CHANNEL_1:
		pin_a = GPIO_PIN_1;
		pin_b = GPIO_PIN_3;
		break;
	case TIM_CHANNEL_2:
		pin_a = GPIO_PIN_4;
		pin_b = GPIO_PIN_5;
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
