/*
 * control_motor.h
 *
 *  Created on: May 28, 2022
 *      Author: bttle
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;

typedef enum
{
	CW = 1, // ruch zgodnie z ruchem wskazówek zegara
	CCW = 0// ruch przeciwnie do rucho wskazówek zegara
}MotorDirection;

typedef struct
{

}MotorPosition;

// q1
void servo_init();
void servo_setAngle(uint8_t angle);
int servo_setDirection(MotorDirection dir);

// q2
void motorA_init();
int motorA_setDirection(MotorDirection dir);
void motorA_setSpeed(uint8_t pwm);
void motorA_stop();

// q3
void motorB_init();
int motorB_setDirection(MotorDirection dir);
void motorB_setSpeed(uint8_t pwm);
void motorB_stop();


#endif /* INC_CONTROL_MOTOR_H_ */
