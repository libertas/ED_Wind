/*
 * ppm.h
 *
 *  Created on: Jul 29, 2017
 *      Author: xita
 */

#ifndef PPM_H_
#define PPM_H_

#include <stdbool.h>

#include "stm32f1xx_hal.h"

#define PPM_PERIOD 2000
#define PPM_CHANNELS 4

void ppm_set(TIM_HandleTypeDef *ppm_htim, uint32_t channel, uint16_t count);

#endif /* PPM_H_ */
