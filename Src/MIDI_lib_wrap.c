


#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ext_line.h"

#define DEFAULT_RESTART_STEPS_VALUE	96

uint8_t start_status = 0;
uint16_t resturt_counter,restart_value = DEFAULT_RESTART_STEPS_VALUE;

extern uint8_t need_start;

extern osThreadId LedTaskHandle;



void MIDI_recive_clock_pulse_handler(void){

	BaseType_t xHigherPriorityTaskWoken;

	put_MIDI_clock();

	start_status++;
	start_status %= restart_value;

	if(!start_status){

		//put_MIDI_start();

		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR( LedTaskHandle,
								( 1UL << 1UL ),
								eSetBits,
								&xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
	else if(!(start_status % 24)){

		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR( LedTaskHandle,
								( 1UL << 0UL ),
								eSetBits,
								&xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}


}


void MIDI_recive_start_handler(void){
	BaseType_t xHigherPriorityTaskWoken;

	if(need_start){
		put_MIDI_start();
		need_start = 0;
	}
	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( LedTaskHandle,
							( 1UL << 1UL ),
							eSetBits,
							&xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	resturt_counter = 0;
	start_status = 0;

}



void start_button_handle(extLine_HandleTypeDef *start_button_extLine_struct){


	if(extLine_get_new_state(start_button_extLine_struct)){
		if(start_button_extLine_struct->extLine_level_status == extLine_level_ZERO){
			//xTaskNotify(LedTaskHandle, ( 1UL << 1UL ), eSetBits );


			need_start = 5;


//			resturt_counter = 0;
//			start_status = 5;
//			put_MIDI_start();
//
//
//			xTaskNotify( LedTaskHandle,
//									( 1UL << 1UL ),
//									eSetBits );


		}
	}

}









