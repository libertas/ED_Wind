/*
 * mwc.c
 *
 *  Created on: Jul 30, 2017
 *      Author: libertas
 */

#include "mwc.h"
#include "ppm.h"

TIM_HandleTypeDef *mwc_ctrl_htim;
TIM_HandleTypeDef *mwc_aux_htim;

void mwc_init(TIM_HandleTypeDef *ctrl_htim, TIM_HandleTypeDef *aux_htim)
{
	mwc_ctrl_htim = ctrl_htim;
	mwc_aux_htim = aux_htim;
}

void mwc_lock()
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_1, 1000);
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_4, 1000);
	ppm_set(mwc_aux_htim, TIM_CHANNEL_1, 1050);

	osDelay(2000);
}

void mwc_unlock()
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_1, 1000);
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_4, 1950);
	ppm_set(mwc_aux_htim, TIM_CHANNEL_1, 1050);

	osDelay(2000);

	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_4, 1500);
}

void mwc_throttle(uint16_t value)
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_1, value);
}

void mwc_roll(uint16_t value)
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_2, value);
}

void mwc_pitch(uint16_t value)
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_3, value);
}

void mwc_yaw(uint16_t value)
{
	ppm_set(mwc_ctrl_htim, TIM_CHANNEL_4, value);
}

void mwc_aux1(uint16_t value)
{
	ppm_set(mwc_aux_htim, TIM_CHANNEL_1, value);
}
