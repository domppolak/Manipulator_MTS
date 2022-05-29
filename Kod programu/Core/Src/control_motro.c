/*
 * control_motro.c
 *
 *  Created on: May 28, 2022
 *      Author: bttle
 */

#include "control_motor.h"

// q1
void servo_init()
{

}

void servo_setAngle(uint8_t angle)
{

}

int servo_setDirection(MotorDirection dir)
{
	return 0;
}

// q2
void motorA_init()
{
	motorA_setDirection(CW);
	motorA_setSpeed(0);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
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

void motorA_setSpeed(uint8_t pwm)
{

}

void motorA_stop()
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

// q3
void motorB_init()
{
	motorA_setDirection(CW);
	motorA_setSpeed(0);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
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

void motorB_setSpeed(uint8_t pwm)
{

}

void motorB_stop()
{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

