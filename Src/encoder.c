/*
 * encoder.c
 *
 *  Created on: 2 мая 2017 г.
 *      Author: koko
 */

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "encoder.h"


extern extLine_HandleTypeDef 	enc01_extLine_struct,
								enc02_extLine_struct;



__weak void encoder_stepup(encoder_HandleTypeDef * enc_struct){

 }

__weak void encoder_stepdown(encoder_HandleTypeDef * enc_struct){

 }



_encoder_states get_encoder_states(encoder_HandleTypeDef * enc_struct){

	if(enc_struct->line01->extLine_level_status){
				if(enc_struct->line02->extLine_level_status){
					return one_one;
				}else{
					return one_zero;
				}
	}else{
				if(enc_struct->line02->extLine_level_status){
					return zero_one;
				}else{
					return zero_zero;
				}
	}
}



void encoder_init(encoder_HandleTypeDef * enc_struct ){

	enc_struct->encoder_states = get_encoder_states(enc_struct);
}

void encoder_handle(encoder_HandleTypeDef * enc_struct){
	_encoder_states new_encoder_states;
	int8_t vv;


	if(extLine_get_new_state(enc_struct->line01)){
		new_encoder_states = get_encoder_states(enc_struct);
	}else if(extLine_get_new_state(enc_struct->line02)){
		new_encoder_states = get_encoder_states(enc_struct);
	}else{
		return;
	}
	vv = new_encoder_states - enc_struct->encoder_states;
	enc_struct->encoder_states = new_encoder_states;
	switch(vv){
		case -3:
		case  1:
			encoder_stepup(enc_struct);
			break;
		case -1:
		case  3:
			encoder_stepdown(enc_struct);
			break;

	}//switch

}

