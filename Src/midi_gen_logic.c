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



#define BUTTON_BLINK_DELAY 100



extern osThreadId LedTaskHandle;
extern UART_HandleTypeDef huart1;
extern osThreadId lamp_TaskHandle;

extern uint8_t uartRX_byte;



extLine_HandleTypeDef 	enc01_extLine_struct,
						enc02_extLine_struct,
						start_button_extLine_struct;

encoder_HandleTypeDef enc01_struct;

uint8_t need_start = 0;

#define WS_LEN 2
uint32_t ws_buff[WS_LEN];
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

void Perf_Task(void){

uint8_t	led_trigger,led_trigger_val;




/* Unlock the Flash to enable the flash control register access *************/
HAL_FLASH_Unlock();

/* Unlock the Options Bytes *************************************************/
HAL_FLASH_OB_Unlock();

FLASH_OBProgramInitTypeDef my_pOBInit;
/* Get pages write protection status ****************************************/
HAL_FLASHEx_OBGetConfig(&my_pOBInit);

/* Check if readoutprtection is enabled ***********************/
if((my_pOBInit.RDPLevel) == OB_RDP_LEVEL_0)
{
	my_pOBInit.OptionType= OPTIONBYTE_RDP;
	my_pOBInit.RDPLevel   = OB_RDP_LEVEL_1;
  if(HAL_FLASHEx_OBProgram(&my_pOBInit) != HAL_OK)
  {
    /* Error occurred while options bytes programming. **********************/
	  Error_Handler();
  }
  /* Generate System Reset to load the new option byte values ***************/
  HAL_FLASH_OB_Launch();
}
/* Lock the Options Bytes *************************************************/
HAL_FLASH_OB_Lock();







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
		  //start_button_handle(&start_button_extLine_struct);
		  int i;
		  for(i=0;i<WS_LEN;i++){
			  ws_buff[i] = 0;
		  }
		  ws_buff[1] = 0xffffffff;

		  	WS2812_buff_send(ws_buff,WS_LEN);

			uint32_t ulNotifiedValue;

			xTaskNotifyWait( 	0x00,      /* Don't clear any notification bits on entry. */
										0xffffffff , /* Reset the notification value to 0 on exit. */
										&ulNotifiedValue, /* Notified value pass out in
														  ulNotifiedValue. */
										portMAX_DELAY );  /* Block indefinitely. */

		  osDelay(100);

//		  if(need_start){
//			  if(led_trigger){
//				  if(!led_trigger_val--){
//					  led_trigger_val = BUTTON_BLINK_DELAY;
//					  led_trigger = 0;
//					  Onboard_led_ON();
//					  lamp01_ON();
//				  }
//			  }else{
//				  if(!led_trigger_val--){
//					  led_trigger_val = BUTTON_BLINK_DELAY;
//					  led_trigger = 5;
//					  Onboard_led_OFF();
//					  lamp01_OFF();
//				  }
//			  }
//		  }//if(need_start){

	  }
}



void lcd_Task(void){

	uint32_t ulNotifiedValue;

		for(;;)
		{
			xTaskNotifyWait( 	0x00,      /* Don't clear any notification bits on entry. */
								0xffffffff , /* Reset the notification value to 0 on exit. */
								&ulNotifiedValue, /* Notified value pass out in
												  ulNotifiedValue. */
								portMAX_DELAY );  /* Block indefinitely. */




						Onboard_led_ON();
						lamp01_ON();
						osDelay(50);
						Onboard_led_OFF();
						lamp01_OFF();
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
				lamp01_ON();
				osDelay(200);
				Onboard_led_OFF();
				lamp01_OFF();
			}

	        if(( ulNotifiedValue & 0x01 ) != 0 )
	        {
	        	Onboard_led_ON();
	        	lamp01_ON();
				osDelay(50);
				Onboard_led_OFF();
				lamp01_OFF();
	        }
	  }
}













