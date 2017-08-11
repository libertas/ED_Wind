/*
 * motion.h
 *
 *  Created on: Jul 22, 2017
 *      Author: xita
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#include "mpu6050.h"

#define MOTOR_ANGLE_LIMIT 0.1f

bool debugFlag;


void motion_init(TIM_HandleTypeDef *htim);

void motor_control(struct kine_state *ks);
void motor_start();
void motor_stop();
void motor_reset();

void motion_control();

void l298n_set(uint32_t channel, float duty);

#endif /* MOTION_H_ */
