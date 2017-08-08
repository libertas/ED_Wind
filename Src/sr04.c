/*
 * sr04.c
 *
 *  Created on: Aug 8, 2017
 *      Author: xita
 */

#include "sr04.h"

TIM_HandleTypeDef *sr04_htim;

uint16_t sr04_count[2] = {0};

void sr04_init(TIM_HandleTypeDef *htim)
{
	sr04_htim = htim;

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 20);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 20);

	HAL_TIM_IC_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_IC_Start(htim, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
}

void sr04_callback(TIM_HandleTypeDef *htim)
{
	if(htim == sr04_htim) {
		sr04_count[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
		sr04_count[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_4);

		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
	}
}

float sr04_get(uint8_t channel)
{
	float res;
	res = (float)sr04_count[channel];
	res = res * 340 / 2 * 1e-6;
	return res;
}
