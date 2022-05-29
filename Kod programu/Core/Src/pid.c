/*
 * pid.c
 *
 */


void pid_init(PidStruct *pid, float kp, float ki, float kd, int limit)
{
	pid->error = 0;
	pid->total_error = 0;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->limit = limit;
}

// pid
// u = kp * e + ki * total_e + kd * (e - e_last);
float pid_calculate(PidStruct *pid, int desire, int current)
{
	float p,i,d;

	int error = desire - current;
	pid->total_error += error;

	p = (float)floatpid->kp * error;
	i = (float)floatpid->ki * pid->total_error;
	d = (float)floatpid->kd * (error - pid->previous_error);

	if(i >= pid->limit){
		i = pid->limit;
	}else if(i <= -pid->limit){
		i = -pid->limit;
	}

	pid->previous_error = error;

	return p+i+d;
}
