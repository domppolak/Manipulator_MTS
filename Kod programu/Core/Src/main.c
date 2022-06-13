/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pid.h"
#include "geometrik.h"
#include "structposition.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_ANGEL 0
#define MAX_ANGEL 180 //1800
#define PWM_MIN 1000
#define PWM_MAX 4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum
{
	CW = 1, // ruch zgodnie z ruchem wskazówek zegara
	CCW = 0// ruch przeciwnie do rucho wskazówek zegara
}MotorDirection;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  int moveFlag = 0;
  float u1, u2;
  int enc_pos1, enc_pos2;
  int16_t count1, count2;
  long int position1, position2;
  PidStruct *pid1, *pid2;
  Position Pos={0,0,0,0,0,0};
  float kp=0.3, ki=1.3, kd=0.5;
  uint8_t znak;
  uint8_t odebrano[15];
  int moveq1 = 0, moveq2 = 0, moveq3 = 0, moveq = 0;
  int moveCoordinates = 0;
  char data[3];
  int value = 20;
  uint16_t katy[7] = {0, 30, 45, 60, 90, 135, 180};
  int16_t pwm_tab[4] = {-100, -50, 50, 100};
  int move = 0;
  int itx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
	return len;
}

void indeks(uint8_t liczba){
	if(liczba == '0'){
		value = 0;
	}
	if(liczba == '1'){
		value = 1;
	}
	if(liczba == '2'){
		value = 2;
	}
	if(liczba == '3'){
		value = 3;
	}
	if(liczba == '4'){
		value = 4;
	}
	if(liczba == '5'){
		value = 5;
	}
	if(liczba == '6'){
		value = 6;
	}
	if(liczba == '7'){
		value = 7;
	}
}

void menu()
{
	printf("Manipulator MTS\n\r");
	HAL_Delay(10);
	printf("q1 %d\n\r", Pos.q1);
	HAL_Delay(10);
	printf("q2 %d\n\r", Pos.q2);
	HAL_Delay(10);
	printf("q3 %d\n\r", Pos.q3);
	HAL_Delay(10);
	printf("X %d\n\r", Pos.x);
	HAL_Delay(10);
	printf("Y %d\n\r", Pos.y);
	HAL_Delay(10);
	printf("Z %d\n\r", Pos.z);
	HAL_Delay(10);
}

void motorA_Direction(MotorDirection dir){
	if(dir == CW){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
	}

	if(dir == CCW){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
	}
}

void motorB_Direction(MotorDirection dir){
	if(dir == CW){
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
	}

	if(dir == CCW){
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
	}
}

void motorA_move(int32_t pwm){
	if(pwm > htim1.Instance->ARR){
		pwm = htim1.Instance->ARR;
	}
	else if(pwm < -htim1.Instance->ARR){
		pwm = -htim1.Instance->ARR;
	}

	if(pwm >= 0){
		motorA_Direction(CW);
	}else{
		motorA_Direction(CCW);
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs(pwm));
}

void motorB_move(int32_t pwm){
	if(pwm > 100){
		pwm = 100;
	}
	else if(pwm < -100){
		pwm = -100;
	}

	if(pwm >= 0){
		motorB_Direction(CW);
	}else{
		motorB_Direction(CCW);
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(pwm));
}

void motorA_stopMotor(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

void motorB_stopMotor(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

// 45 = 450
// step ((1000*(PWM_MAX - PWM_MIN) / (MAX_ANGEL - MIN_ANGEL));
void servo_moveAngel(uint16_t angel, MotorDirection dir){
	uint16_t pwm;
	if(angel > MAX_ANGEL){
		angel = MAX_ANGEL;
	}
	else if(angel < MIN_ANGEL){
		angel = MIN_ANGEL;
	}

	if(dir == CW){
		pwm = PWM_MIN + (angel*(PWM_MAX - PWM_MIN)/MAX_ANGEL); // excel mowi ze wylicza dobrze
		// pwm = PWM_MIN + ((angel - MIN_ANGEL) * step) / 1000;
	}
	else{
		pwm = PWM_MAX - (angel*(PWM_MAX - PWM_MIN)/MAX_ANGEL);
	}

	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pwm + 50);
}
/*void enc_position2(int16_t *position)
{
	position = ((int16_t)(htim4.Instance->CNT))/4;
}*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &znak, 1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //servo   		q3
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // silnik 1 	q1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // silnik 2		q2
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1000); // serwo pozycja startowa
  //pid_init(pid1, kp, ki, kd); / silnik 1
  pid_init(pid2, kp, ki, kd);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //silnik 2
  //HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //silnik 1

	//HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  //motorB_Direction(CW);
  /*for(int i=0; i<20; i++){
	  motorB_move(50-i);
	  HAL_Delay(10);

  }
  HAL_Delay(200);
  motorB_stopMotor();*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  menu();
  while (1)
  {
	  if(moveFlag == 1){
		  if(moveq == 1){
			  if(moveq2 == 1 && move == 1){
				  for(int i=0; i<15; i++){
					  if(pwm_tab[value]>0)
						  motorB_Direction(CW);
					  else{
						  motorB_Direction(CCW);
					  }
					  HAL_Delay(250);
					  motorB_move(abs(pwm_tab[value]));
					  moveq2 = 0;
					  moveq = 0;
					  move = 0;
				  }
				  HAL_Delay(200);
				  motorB_stopMotor();
			  }
			  if(moveq3 == 1 && move == 1){
					 servo_moveAngel(katy[value], CW);
					 moveq3 = 0;
					 move = 0;
			  }
		  }
		  moveFlag = 0;
	  }
	  //menu();
	  //HAL_UART_Receive(&huart2, &znak, 1, 50);

		 /* servo_moveAngel(0, CW);
		  HAL_Delay(1000);
		  servo_moveAngel(90, CW);
		  HAL_Delay(1000);*/
		  //servo_moveAngel(45, CW);
		  //HAL_Delay(1000);
		  //servo_moveAngel(90, CW);
		  //HAL_Delay(1000);
		  //servo_moveAngel(0, CCW);
		  //HAL_Delay(1000);
		  //menu();

	  /*__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1000); //0
	  HAL_Delay(1000);
	  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1500); //45
	  HAL_Delay(1000);
	  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 2000); //90
	  HAL_Delay(1000);*/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(&huart2, &znak, 1, 100);
	itx++;
	if(znak == 'm'){
		moveFlag = 1;
	}
	if(znak == 'q'){
		moveq = 1;
	}
	if(znak == 'a'){
		moveq1 = 1;
	}
	if(znak == 'b'){
		moveq2 = 1;
	}
	if(znak == 'c'){
		moveq3 = 1;
	}

	if(moveFlag == 1 && moveq == 1 && moveq2 == 1 && itx == 4){
		indeks(znak);
		move = 1;
		itx = 0;
	}
	if(moveFlag == 1 && moveq == 1 && moveq3 == 1 && itx == 4){
		indeks(znak);
		move = 1;
		itx = 0;
	}

	/*char *tmp;
	if(data[0] == 'm'){
		moveFlag = 1;
	}
	if(data[1] == 'q'){
		if(data[2] == '1'){
			moveq1 = 1;
			tmp = strtok(data, " ");
			tmp = strtok(data, " ");
			tmp = tmp+1;
			Pos.q1 = atoi(tmp);
		}
		if(data[2] == '2'){
			moveq2 = 1;
			tmp = strtok(data, " ");
			tmp = tmp+1;
			Pos.q2 = atoi(tmp);
		}
		if(data[2] == '3'){
			moveq3 = 1;
			tmp = strtok(data, " ");
			tmp = tmp+1;
			Pos.q3 = atoi(tmp);
		}
	}

	if(data[0] == 'c'){
		moveCoordinates = 1;
	}
*/
	HAL_UART_Receive_IT(&huart2, &znak, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
