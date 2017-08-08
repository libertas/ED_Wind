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
void sr04_callback();

#endif /* SR04_H_ */
