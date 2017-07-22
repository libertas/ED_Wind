#include "pwm.h"

void set_duty(TIM_HandleTypeDef *htim, uint32_t Channel, float duty)
{
	__HAL_TIM_SET_COMPARE(htim, Channel, (uint32_t)(duty * PWM_PERIOD));
}
