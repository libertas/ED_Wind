/*
 * motion.h
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "stm32f4xx_hal.h"

#include "mpu6050.h"

void motion_init(TIM_HandleTypeDef *htim);

void motion_control();

void l298n_set(uint32_t channel, float duty);

#endif /* MOTION_H_ */
