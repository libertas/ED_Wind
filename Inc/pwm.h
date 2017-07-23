/*
 * pwm.h
 *
 *  Created on: Jul 20, 2017
 *      Author: xita
 */

#ifndef PWM_H_
#define PWM_H_

#define PWM_PERIOD 1000

#include "stm32f1xx_hal.h"

inline void set_duty(TIM_HandleTypeDef *htim, uint32_t Channel, float duty);

#endif /* PWM_H_ */