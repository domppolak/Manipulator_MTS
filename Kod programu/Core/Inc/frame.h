/*
 * frame.c
 *
 *  Created on: May 16, 2022
 *      Author: bttle
 */

#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "structposition.h"
#include "main.h"



typedef struct{
	const uint8_t *start;
	const uint8_t *current;
	const uint8_t *end;

}DataFrame;

void frame_init(DataFrame *frame);
void frame_recv_char(DataFrame *frame);

// [Head][LENGTH][Data][CRC]
void frame_send_data(Position *position, UART_HandleTypeDef *huart);
#endif /* INC_FRAME_H_ */
