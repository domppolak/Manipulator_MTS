/*
 * frame.c
 *
 *  Created on: May 16, 2022
 *      Author: bttle
 */

#include"frame.h"
#include "usart.h"
#include <string.h>

void frame_recv_char(DataFrame *frame){

}

void create_frame(Position *position, uint8_t *buffer){

}

void frame_send_data(Position *position, UART_HandleTypeDef *huart){

	static uint8_t Data[15];

	HAL_UART_Transmit_DMA(huart, Data, 15);
	HAL_Delay(1);
}
