/*
 * pid.c
 *
 */

#include "pid.h"

#define ERR_TOTAL_MAX 255

void pid_init(PidStruct *pid, float kp, float ki, float kd, int limit)
{
	pid->previous_error = 0;
	pid->total_error = 0;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

}

// pid
// u = kp * e + ki * total_e + kd * (e - e_previous);
float pid_calculate(PidStruct *pid, int desire, int current)
{
	float p,i,d;

	int error = desire - current;
	pid->total_error += error;



	if(pid->total_error > ERR_TOTAL_MAX){
		pid->total_error = ERR_TOTAL_MAX;
	}else if(pid->total_error < -ERR_TOTAL_MAX){
		pid->total_error = -ERR_TOTAL_MAX;
	}

	p = (float)(pid->kp * error);
	i = (float)(pid->ki * pid->total_error);
	d = (float)(pid->kd * (error - pid->previous_error));

	pid->previous_error = error;

	return p+i+d;
}
