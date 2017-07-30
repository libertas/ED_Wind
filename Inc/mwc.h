/*
 * mwc.h
 *
 *  Created on: Jul 30, 2017
 *      Author: libertas
 */

#ifndef MWC_H_
#define MWC_H_

#include "stm32f1xx_hal.h"

void mwc_init(TIM_HandleTypeDef *ctrl_htim, TIM_HandleTypeDef *aux_htim);
void mwc_unlock();
void mwc_throttle(uint16_t value);
void mwc_roll(uint16_t value);
void mwc_pitch(uint16_t value);
void mwc_yaw(uint16_t value);
void mwc_aux1(uint16_t value);


#endif /* MWC_H_ */
