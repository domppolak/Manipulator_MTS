/*
 * control_motro.c
 *
 *  Created on: May 28, 2022
 *      Author: bttle
 */

#include "control_motor.h"
#include <stdlib.h>

TIM_HandleTypeDef *timA;
TIM_HandleTypeDef *timB;
TIM_HandleTypeDef *timServo;
uint32_t tim_channelA;
uint32_t tim_channelB;
uint32_t tim_channelServo;

// q1
void servo_init(TIM_HandleTypeDef *tim, uint32_t channel)
{
	timServo = tim;
	tim_channelServo = channel;
	HAL_TIM_PWM_Start(timServo, tim_channelServo);

	__HAL_TIM_SET_COMPARE(timServo, tim_channelServo, 1050);
	HAL_Delay(1000);
}

void servo_move(uint16_t angel, MotorDirection dir)
{
	int pwm;
	if(angel > MAX_ANGEL){
		angel = MAX_ANGEL;
	}
	else if(angel < MIN_ANGEL){
		angel = MIN_ANGEL;
	}

	if(dir){
		pwm = PWM_MIN + ((angel - MIN_ANGEL) * STEP) / 1000;
	}
	else{
		pwm = PWM_MAX - ((angel - MIN_ANGEL) * STEP) / 1000;
	}

	__HAL_TIM_SET_COMPARE(timServo, tim_channelServo, pwm+50);
}

// q2
void motorA_init(TIM_HandleTypeDef *tim, uint32_t tim_channel)
{
	timA = tim;
	tim_channelA = tim_channel;
	motorA_setDirection(CW);
	motorA_setSpeed(0);
	HAL_TIM_PWM_Start(timA, tim_channelA);
}

void motorA_structInit(MotorStruct *m, TIM_HandleTypeDef *tim)
{
	m->timer = tim;
	m->resolution = ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGET1T2 * MOTOR_GEAR;

	m->pulse_count = 0;
	m->speed = 0;
}

int motorA_setDirection(MotorDirection dir)
{
	if(dir == CW){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		return 1;
	}

	if(dir == CCW){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		return 1;
	}

	return 0;
}

void motorA_setSpeed(int pwm)
{
	if(pwm > 100){
		pwm = 100;
	}
	else if(pwm < -100){
		pwm = -100;
	}

	if(pwm >= 0){
		motorA_setDirection(CW);
		__HAL_TIM_SET_COMPARE(timA, tim_channelA, pwm);
	}
	else{
		motorA_setDirection(CCW);
		__HAL_TIM_SET_COMPARE(timA, tim_channelA, abs(pwm));
	}
}

void motorA_stop()
{
	//HAL_TIM_PWM_Stop(timA, tim_channelA);
	motorA_setSpeed(0);
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

// q3
void motorB_init(TIM_HandleTypeDef *tim, uint32_t tim_channel)
{
	timB = tim;
	tim_channelB = tim_channel;
	motorA_setDirection(CW);
	motorA_setSpeed(0);
	HAL_TIM_PWM_Start(timB, tim_channelB);
}

void motorB_structInit(MotorStruct *m, TIM_HandleTypeDef *tim)
{
	m->timer = tim;
	m->resolution = ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGET1T2 * MOTOR_GEAR;

	m->pulse_count = 0;
	m->speed = 0;
}

int motorB_setDirection(MotorDirection dir)
{
	if(dir == CW){
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		return 1;
	}

	if(dir == CCW){
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		return 1;
	}

	return 0;
}

void motorB_setSpeed(int pwm)
{
	if(pwm > 100){
		pwm = 100;
	}
	else if(pwm < -100){
		pwm = -100;
	}

	if(pwm >= 0){
		motorA_setDirection(CW);
		__HAL_TIM_SET_COMPARE(timA, tim_channelA, pwm);
	}
	else{
		motorA_setDirection(CCW);
		__HAL_TIM_SET_COMPARE(timA, tim_channelA, abs(pwm));
	}
}

void motorB_stop()
{
	//HAL_TIM_PWM_Stop(timB, tim_channelB);
	motorB_setSpeed(0);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

