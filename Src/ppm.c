/*
 * ppm.c
 *
 *  Created on: Jul 29, 2017
 *      Author: xita
 */

#include "ppm.h"

TIM_HandleTypeDef *ppm_htim;

void ppm_init(TIM_HandleTypeDef *htim, uint16_t data[PPM_CHANNELS])
{
	ppm_htim = htim;

	ppm_set(data);

	HAL_TIM_PWM_Start(ppm_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(ppm_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(ppm_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(ppm_htim, TIM_CHANNEL_4);

	HAL_TIM_Base_Start_IT(ppm_htim);
}

void ppm_set(uint16_t data[PPM_CHANNELS])
{
	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_1, data[0]);
	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_2, data[1]);
	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_3, data[2]);
	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_4, data[3]);
}
