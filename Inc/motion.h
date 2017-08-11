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

#define MOTOR_LIMIT 800
#define MOTOR_MID 2500

bool debugFlag;

uint16_t get_pos_x();
uint16_t get_pos_y();

void motion_init(TIM_HandleTypeDef *htim);

void motor_control();
void motor_start();
void motor_stop();
void motor_reset();
void motor_move_mid();

void motion_control();

void move_to_pos(uint16_t x, uint16_t y);

void l298n_set(uint32_t channel, float duty);

#endif /* MOTION_H_ */
