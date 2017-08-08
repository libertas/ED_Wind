/*
 * sr04.h
 *
 *  Created on: Aug 8, 2017
 *      Author: xita
 */

#ifndef SR04_H_
#define SR04_H_

#include "stm32f4xx_hal.h"

#define ULTRASONIC_PERIOD 10000

void sr04_init(TIM_HandleTypeDef *htim);
float sr04_get(uint8_t channel);

#endif /* SR04_H_ */
