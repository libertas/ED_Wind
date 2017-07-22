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

	px.kp = 1;
	px.ki = 0;
	px.kd = 0;

	py.kp = 1;
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

	set_duty(motion_htim, TIM_CHANNEL_1, vx);
	set_duty(motion_htim, TIM_CHANNEL_2, -vx);
	set_duty(motion_htim, TIM_CHANNEL_3, vy);
	set_duty(motion_htim, TIM_CHANNEL_4, -vy);
}
