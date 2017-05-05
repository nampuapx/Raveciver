/*
 * ext_line.c
 *
 *  Created on: 2 мая 2017 г.
 *      Author: koko
 */

#include "stm32f1xx_hal.h"
#include "ext_line.h"



#define ANTI_DREBEZG_VALUE  5

#define ANTI_DREBEZG_VALUE_MIDDLE  127

void extLine_init(	extLine_HandleTypeDef * extLine_Handle_struct,
					GPIO_TypeDef* GPIOx,
					uint16_t GPIO_Pin){

	extLine_Handle_struct->line_state_counter = ANTI_DREBEZG_VALUE_MIDDLE;
	extLine_Handle_struct->extLine_states = extLine_processing_UNDONE;
	extLine_Handle_struct->GPIOx = GPIOx;
	extLine_Handle_struct->GPIO_Pin = GPIO_Pin;
	extLine_Handle_struct->extLine_level_status = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

}




uint8_t extLine_get_new_state(	extLine_HandleTypeDef * extLine_Handle_struct){

	uint8_t ext=0;

	if(HAL_GPIO_ReadPin(extLine_Handle_struct->GPIOx, extLine_Handle_struct->GPIO_Pin)){
		if(extLine_Handle_struct->extLine_level_status){
			//return (0);
			ext = 0;
		}else{
			extLine_Handle_struct->line_state_counter++;
			if(extLine_Handle_struct->line_state_counter > (ANTI_DREBEZG_VALUE_MIDDLE + ANTI_DREBEZG_VALUE)){
				extLine_Handle_struct->line_state_counter = (ANTI_DREBEZG_VALUE_MIDDLE + ANTI_DREBEZG_VALUE);
				extLine_Handle_struct->extLine_level_status = extLine_level_FULL;
				//return (5);
				ext = 5;
			}
		}
	}else{

		if(extLine_Handle_struct->extLine_level_status){

			extLine_Handle_struct->line_state_counter--;
			if(extLine_Handle_struct->line_state_counter < (ANTI_DREBEZG_VALUE_MIDDLE - ANTI_DREBEZG_VALUE)){
				extLine_Handle_struct->line_state_counter = (ANTI_DREBEZG_VALUE_MIDDLE - ANTI_DREBEZG_VALUE);
				extLine_Handle_struct->extLine_level_status = extLine_level_ZERO;
				//return (5);
				ext = 5;
			}
		}else{
				//return (0);
				ext = 0;
		}
	}
	return ext;
}



