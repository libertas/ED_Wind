/*
 * ppm.c
 *
 *  Created on: Jul 29, 2017
 *      Author: xita
 */

#include "ppm.h"

void ppm_set(TIM_HandleTypeDef *ppm_htim, uint32_t channel, uint16_t count)
{
	__HAL_TIM_SET_COMPARE(ppm_htim, channel, count);

	HAL_TIM_PWM_Start(ppm_htim, channel);

	HAL_TIM_Base_Start_IT(ppm_htim);
}
