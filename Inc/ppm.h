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
#define PPM_CHANNELS 10

void ppm_init(TIM_HandleTypeDef *htim);
bool ppm_send(uint16_t data[PPM_CHANNELS]);
void ppm_callback();

#endif /* PPM_H_ */
