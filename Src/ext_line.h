/*
 * ext_line.h
 *
 *  Created on: 2 мая 2017 г.
 *      Author: koko
 */

#ifndef EXT_LINE_H_
#define EXT_LINE_H_


typedef enum {
	extLine_processing_UNDONE,
	extLine_processing_DONE
}_extLine_processing_states;

typedef enum {
	extLine_level_ZERO = 0,
	extLine_level_FULL
}_extLine_level_states;


typedef struct
{
	uint8_t line_state_counter;
	_extLine_processing_states extLine_states;
	_extLine_level_states extLine_level_status;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}extLine_HandleTypeDef;




void extLine_init(	extLine_HandleTypeDef * extLine_Handle_struct,
					GPIO_TypeDef* GPIOx,
					uint16_t GPIO_Pin);



uint8_t extLine_get_new_state(	extLine_HandleTypeDef * extLine_Handle_struct);





#endif /* EXT_LINE_H_ */
