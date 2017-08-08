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

	__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,\
			TIM_INPUTCHANNELPOLARITY_RISING);


	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static state = 0;

	static tmp0 = 0;

	if(htim == sr04_htim) {
		if(__HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2) != 0) {
			if(state == 0) {
				state++;

				tmp0 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,\
						TIM_INPUTCHANNELPOLARITY_FALLING);
			} else {
				state = 0;

				sr04_count[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2)\
						- tmp0;

				tmp0 = 0;

				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,\
						TIM_INPUTCHANNELPOLARITY_RISING);
			}
		}

		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);

		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
	}
}

float sr04_get(uint8_t channel)
{
	float res;
	res = sr04_count[channel] * 340 / 2 * 1e-6;
	return res;
}
