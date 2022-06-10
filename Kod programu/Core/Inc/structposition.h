/*
 * position.h
 *
 *  Created on: May 16, 2022
 *      Author: bttle
 */

#ifndef INC_STRUCTPOSITION_H_
#define INC_STRUCTPOSITION_H_
#include "main.h"

typedef struct {
	int16_t q1=0;
	int16_t q2=0;
	int16_t q3=0;

	int16_t x=0;
	int16_t y=0;
	int16_t z=0;
}Position;

#endif /* INC_STRUCTPOSITION_H_ */
