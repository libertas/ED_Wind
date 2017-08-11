/*
 * resistor.c
 *
 *  Created on: Aug 11, 2017
 *      Author: xita
 */

#include "resistor.h"

ADC_HandleTypeDef *resistor_hadc;

uint32_t resistor_dma_data[4] = {0};

void resistor_init(ADC_HandleTypeDef *hadc)
{
	resistor_hadc = hadc;
}

void resistor_start_dma()
{
	HAL_ADC_Start_DMA(resistor_hadc, resistor_dma_data, 4);
}

uint16_t resistor_get(uint8_t channel)
{
	return resistor_dma_data[channel];
}
