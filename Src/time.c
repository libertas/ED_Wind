/*
 * time.c
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#include <stdint.h>

#include "time.h"

TIM_HandleTypeDef *time_htim;

uint32_t time_counter = 10;

void time_init(TIM_HandleTypeDef *htim)
{
	time_htim = htim;
	HAL_TIM_Base_Start_IT(htim);
}

void time_callback()
{
	time_counter++;
}

float micros()
{
	return (float)(time_counter * MICROS_PER_TICK);
}

float millis()
{
	return (float)(time_counter * MILLIS_PER_TICK);
}

float seconds()
{
	return (float)(time_counter * SECONDS_PER_TICK);
}
