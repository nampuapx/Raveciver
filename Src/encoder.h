/*
 * encoder.h
 *
 *  Created on: 2 мая 2017 г.
 *      Author: koko
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "ext_line.h"

#define ENC_STEP_AMOUNT 4

typedef enum {
	zero_zero=0,
	zero_one,
	one_one,
	one_zero
}_encoder_states;

typedef struct
{
	_encoder_states encoder_states;
	extLine_HandleTypeDef *line01;
	extLine_HandleTypeDef *line02;
}encoder_HandleTypeDef;

#endif /* ENCODER_H_ */
