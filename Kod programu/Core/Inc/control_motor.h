/*
 * control_motor.h
 *
 *  Created on: May 28, 2022
 *      Author: bttle
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include "main.h"


#define ENCODER_RESOLUTION 				16
#define TIMER_CONF_BOTH_EDGET1T2 		4
#define MOTOR_GEAR						120

#define TIMER_FREQENCY					10
#define SECOND_IN_MINUTE				60

#define MIN_ANGEL 0
#define MAX_ANGEL 90
#define MAX_SPEED 100

#define PWM_MIN 1000
#define PWM_MAX 2000

#define STEP ((1000 * (PWM_MAX - PWM_MIN)) / (MAX_ANGEL - MIN_ANGEL))


/*extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;*/

typedef enum
{
	CW = 1, // ruch zgodnie z ruchem wskazówek zegara
	CCW = 0// ruch przeciwnie do rucho wskazówek zegara
}MotorDirection;

typedef struct
{
	TIM_HandleTypeDef *timer;
	uint16_t resolution;

	int32_t pulse_count;
	int32_t speed;

}MotorStruct;

// q1
void servo_init(TIM_HandleTypeDef *tim, uint32_t channel);
void servo_move(uint16_t angel, MotorDirection dir);

//void servo_setAngle(uint8_t angle);
//int servo_setDirection(MotorDirection dir);

// q2
void motorA_init(TIM_HandleTypeDef *tim, uint32_t tim_channel);
void motorA_structInit(MotorStruct *m, TIM_HandleTypeDef *tim);
int motorA_setDirection(MotorDirection dir);
void motorA_setSpeed(int pwm);
void motorA_stop();

// q3
void motorB_init(TIM_HandleTypeDef *tim, uint32_t tim_channel);
void motorB_structInit(MotorStruct *m, TIM_HandleTypeDef *tim);
int motorB_setDirection(MotorDirection dir);
void motorB_setSpeed(int pwm);
void motorB_stop();


#endif /* INC_CONTROL_MOTOR_H_ */
