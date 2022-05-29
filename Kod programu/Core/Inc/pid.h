/*
 * pid.h
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	float kp;
	float ki;
	float kd;

	int previous_error;
	int total_error;
	int limit; // ograniczenie członu całkującego
}PidStruct;

void pid_init(PidStruct *pid, float kp, float ki, float kd, int limit);
float pid_calculate(PidStruct *pid, int desire, int current);

#endif /* INC_PID_H_ */
