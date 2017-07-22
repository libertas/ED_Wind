/*
 * time.h
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#ifndef TIME_H_
#define TIME_H_

#include "stm32f1xx_hal.h"

#define MICROS_PER_TICK 1000.0f
#define MILLIS_PER_TICK (MICROS_PER_TICK * 0.001)
#define SECONDS_PER_TICK (MICROS_PER_TICK * 0.000001)

void time_init(TIM_HandleTypeDef *htim);
void time_callback();
float micros();
float millis();
float seconds();


#endif /* TIME_H_ */
