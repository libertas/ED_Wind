#include <stdlib.h>
#include <stdint.h>
#include "pid.h"


void pid_config(mypid_t* pid_initstruct)
{
	pid_initstruct->actual_value = 0.0;
	pid_initstruct->error = 0.0;
	pid_initstruct->error_old = 0.0;
	pid_initstruct->sum = 0.0;
}

float pid_realize(mypid_t* pid)
{
    int8_t index = 0;
    if(abs(pid->error) > (pid->set_value * 1.5F)) {
        index = 0;
    }
    else {
        index = 1;
        pid->sum += pid->error;
    }
	pid->error = pid->set_value - pid->actual_value;
	float out = pid->kp * (pid->error)\
              + pid->ki * (pid->sum) * (index)\
              + pid->kd * (pid->error - pid->error_old);
	pid->error_old = pid->error;
	return out;
}



void pid_inc_config(pid_inc_t* pid_inc_initstruct)
{
	pid_inc_initstruct->actual_value = 0.0;
	pid_inc_initstruct->error = 0.0;
	pid_inc_initstruct->error_oold = 0.0;
	pid_inc_initstruct->error_old = 0.0;
	pid_inc_initstruct->sum = 0.0;
}

float pid_inc_realize(pid_inc_t* pid)
{
    pid->error = pid->set_value - pid->actual_value;
    float increment = pid->kp * (pid->error - pid->error_old)\
                    + pid->ki * (pid->error)\
                    + pid->kd * (pid->error - 2 * pid->error_old + pid->error_oold);
    pid->out += increment;
    pid->error_oold = pid->error_old;
    pid->error_old = pid->error;
	return pid->out;
}
