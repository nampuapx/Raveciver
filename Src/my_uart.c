/*
 * my_uart.c
 *
 *  Created on: 4 мая 2017 г.
 *      Author: koko
 */

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;

uint8_t start_arm = 0, clock_arm = 0;

#define MIDI_REAL_TIME_OUT_STACK_SIZE 10
uint8_t midi_real_time_out_stack[MIDI_REAL_TIME_OUT_STACK_SIZE];
uint8_t to_MRTOS_read=0,to_MRTOS_write=0;

uint8_t uartTX_byte;
uint8_t uartRX_byte;



void little_buff_add(uint8_t in){
	uint8_t tt;

	  midi_real_time_out_stack[to_MRTOS_write] = in;
	  tt = to_MRTOS_write;
	  to_MRTOS_write++;
	  to_MRTOS_write %= MIDI_REAL_TIME_OUT_STACK_SIZE;
	  if(to_MRTOS_write == to_MRTOS_read)to_MRTOS_write = tt;
}


void put_MIDI_real_time(uint8_t real_time_command){
	  uint32_t tmp_state = 0;

	  tmp_state = huart1.State;
	  if((tmp_state == HAL_UART_STATE_READY) || (tmp_state == HAL_UART_STATE_BUSY_RX)){

		uartTX_byte = real_time_command;
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&uartTX_byte, 1);

	  }else{
		  little_buff_add(real_time_command);
	  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  uint32_t tmp_state = 0;


	/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  if(to_MRTOS_read != to_MRTOS_write){

	  tmp_state = huart1.State;
	  if((tmp_state == HAL_UART_STATE_READY) || (tmp_state == HAL_UART_STATE_BUSY_RX)){

		uartTX_byte = midi_real_time_out_stack[to_MRTOS_read];
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&uartTX_byte, 1);
		to_MRTOS_read++;
		to_MRTOS_read %= MIDI_REAL_TIME_OUT_STACK_SIZE;
	  }
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

uint8_t rr;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  rr = uartRX_byte;
//  if (HAL_UART_Receive_IT(huart, (uint8_t *)&uartRX_byte, 1) != HAL_OK) {
//      Error_Handler();
//  }
 // midi_parser_byte(rr);

  midi_parser_byte(rr);

}



void  my_UART_Receive_IT(UART_HandleTypeDef *huart)
{
//  uint16_t* tmp;
//  uint32_t tmp_state = 0;
//
//  tmp_state = huart->State;
//  if((tmp_state == HAL_UART_STATE_BUSY_RX) || (tmp_state == HAL_UART_STATE_BUSY_TX_RX))
//  {


//      if(huart->Init.Parity == UART_PARITY_NONE)
//      {
		*huart->pRxBuffPtr =  (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
		HAL_UART_RxCpltCallback(huart);
//      }
//      else
//      {
//        *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
//      }
//

//    if(--huart->RxXferCount == 0)
//    {
//      __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
//
//      /* Check if a transmit process is ongoing or not */
//      if(huart->State == HAL_UART_STATE_BUSY_TX_RX)
//      {
//        huart->State = HAL_UART_STATE_BUSY_TX;
//      }
//      else
//      {
//        /* Disable the UART Parity Error Interrupt */
//        __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
//
//        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//        __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
//
//        huart->State = HAL_UART_STATE_READY;
//      }
//      HAL_UART_RxCpltCallback(huart);
//
//      return HAL_OK;
//    }
//    return HAL_OK;
//  }
//  else
//  {
//    return HAL_BUSY;
//  }
}




