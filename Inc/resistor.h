/*
 * resistor.h
 *
 *  Created on: Aug 11, 2017
 *      Author: xita
 */

#ifndef RESISTOR_H_
#define RESISTOR_H_

#include "stm32f4xx_hal.h"


void resistor_init(ADC_HandleTypeDef *hadc);
void resistor_start_dma();
uint16_t resistor_get(uint8_t channel);


#endif /* RESISTOR_H_ */
