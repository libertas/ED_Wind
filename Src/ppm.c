/*
 * ppm.c
 *
 *  Created on: Jul 29, 2017
 *      Author: xita
 */

#include "ppm.h"

TIM_HandleTypeDef *ppm_htim;
uint16_t *ppm_data;
bool ppm_busy = false;
uint8_t ppm_counter = 0;

void ppm_init(TIM_HandleTypeDef *htim)
{
	ppm_htim = htim;
	ppm_busy = false;

	HAL_TIM_Base_Start_IT(ppm_htim);
}

bool ppm_send(uint16_t data[PPM_CHANNELS])
{
	if(ppm_busy) {
		return false;
	}

	ppm_busy = true;

	ppm_data = data;

	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_1, PPM_PERIOD - ppm_data[0]);
	__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_2, PPM_PERIOD - ppm_data[0]);

	HAL_TIM_OnePulse_Start(ppm_htim, TIM_CHANNEL_1);
	HAL_TIM_OnePulse_Start(ppm_htim, TIM_CHANNEL_2);

	ppm_counter = 1;

	return true;
}

void ppm_callback()
{
	if(ppm_counter >= PPM_CHANNELS) {
		__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_1, PPM_PERIOD);
		__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_2, PPM_PERIOD);
		ppm_busy = false;
	} else {
		__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_1, PPM_PERIOD - ppm_data[ppm_counter]);
		__HAL_TIM_SET_COMPARE(ppm_htim, TIM_CHANNEL_2, PPM_PERIOD - ppm_data[ppm_counter]);

		HAL_TIM_OnePulse_Start(ppm_htim, TIM_CHANNEL_1);
		HAL_TIM_OnePulse_Start(ppm_htim, TIM_CHANNEL_2);

		ppm_counter++;
	}
}
