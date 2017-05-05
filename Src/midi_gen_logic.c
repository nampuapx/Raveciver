/*******************************************************************************
 *
 * midi_gen: is midi_gen
 *
 *    midi_gen:                      copyright 2017       Vetrov Nikolay.
 *
 *    Permission to use, copy, modify, and distribute this software in source
 *    and binary forms and its documentation for any purpose and without fee
 *    is hereby granted, provided that the above copyright notice appear
 *    in all copies and that both that copyright notice and this permission
 *    notice appear in supporting documentation.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE VETROV NIKOLAY AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL VETROV NIKOLAY OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *    THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
//#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ext_line.h"
#include "encoder.h"
#include "MIDI_lib\midi_lib.h"

extern osThreadId LedTaskHandle;
extern UART_HandleTypeDef huart1;
extern osThreadId lamp_TaskHandle;

extern uint8_t uartRX_byte;



extLine_HandleTypeDef 	enc01_extLine_struct,
						enc02_extLine_struct,
						start_button_extLine_struct;

encoder_HandleTypeDef enc01_struct;


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

void Perf_Task(void){

	extLine_init(&start_button_extLine_struct, button01_GPIO_Port, button01_Pin);

	extLine_init(&enc01_extLine_struct, enc01_GPIO_Port, enc01_Pin);
	extLine_init(&enc01_extLine_struct, enc02_GPIO_Port, enc02_Pin);

	enc01_struct.line01 = &enc01_extLine_struct;
	enc01_struct.line02 = &enc02_extLine_struct;



    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&uartRX_byte, 1) != HAL_OK) {
        Error_Handler();
    }


	  for(;;)
	  {
		  //encoder_handle(&enc01_struct);
		  start_button_handle(&start_button_extLine_struct);
		  osDelay(1);
	  }
}



void lamp_Task(void){

	uint32_t ulNotifiedValue;

		for(;;)
		{
			xTaskNotifyWait( 	0x00,      /* Don't clear any notification bits on entry. */
								0xffffffff , /* Reset the notification value to 0 on exit. */
								&ulNotifiedValue, /* Notified value pass out in
												  ulNotifiedValue. */
								portMAX_DELAY );  /* Block indefinitely. */



//
//		HAL_GPIO_WritePin(lamp_01_GPIO_Port, lamp_01_Pin, GPIO_PIN_SET);
//		osDelay(100);
//		HAL_GPIO_WritePin(lamp_01_GPIO_Port, lamp_01_Pin, GPIO_PIN_RESET);
						Onboard_led_ON();
						osDelay(20);
						Onboard_led_OFF();
		}
}

void TIM1_PeriodElapsedCallback(void){

	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( LedTaskHandle,
							( 1UL << 0UL ),
	                        eSetBits,
	                        &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}







void led_task(void){

	uint32_t ulNotifiedValue;

	for(;;)
	  {
			xTaskNotifyWait( 	0x00,      /* Don't clear any notification bits on entry. */
								0xffffffff , /* Reset the notification value to 0 on exit. */
								&ulNotifiedValue, /* Notified value pass out in
												  ulNotifiedValue. */
								portMAX_DELAY );  /* Block indefinitely. */

			if(( ulNotifiedValue & 0x02 ) != 0 ){
				Onboard_led_ON();
				osDelay(200);
				Onboard_led_OFF();
			}

	        if(( ulNotifiedValue & 0x01 ) != 0 )
	        {
	        	Onboard_led_ON();
				osDelay(10);
				Onboard_led_OFF();
	        }
	  }
}













