
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


extern SPI_HandleTypeDef hspi1;
extern osThreadId Perf_TaskHandle;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( Perf_TaskHandle,
							( 1UL << 0UL ),
	                        eSetBits,
	                        &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}


//#ifdef qqq

extern osMutexId winc_spi_mutex_id;
extern 	osThreadId spi_thread_id;



const uint8_t WS_decode[] = {0x88,0x8C,0xC8,0xCC};

#define WS2812_LENGTH 10
char WS2812_buff[WS2812_LENGTH*12];

//int WS2812_SPI3_Init(void)
//{
//    int rc;
//
////    rc = Driver_SPI3.Control(ARM_SPI_MODE_MASTER |
////                        ARM_SPI_SS_MASTER_SW |
////                        ARM_SPI_CPOL0_CPHA1 |
////                        ARM_SPI_MSB_LSB |
////                        ARM_SPI_DATA_BITS(8), 5000000);
////
//
//
//    return rc;
//}

void WS2812_spi_send(char * buff,uint16_t len){
	
	
	HAL_SPI_Transmit_DMA(&hspi1, buff, len);
//	//static char bb[] = {0xa0,0xaa,0x55,0x81};
//	osEvent evt;
//
//  osPriority priority;
//	spi_thread_id = osThreadGetId ();
//	priority = osThreadGetPriority (spi_thread_id);
//
//		osMutexWait(winc_spi_mutex_id, osWaitForever);
//				osThreadSetPriority(spi_thread_id,osPriorityHigh);
//
//
//
//	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//				__nop();
//				__nop();
//				__nop();
//				__nop();
//
//				Driver_SPI3.Send(buff, len);
//
//				//LED_on(LED_BLUE);
//				while(WINC1500_SPI.GetStatus().busy);
//				//evt = osSignalWait(0x0100, 1000);
//        //if (evt.status == osEventTimeout) {
//         //   __breakpoint(0); /* Timeout error: Call debugger */
//        //}
//
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//
//		osThreadSetPriority(spi_thread_id,priority);
//
//		osMutexRelease(winc_spi_mutex_id);
//
//		//evt = osSignalWait(SEND_DONE_SIG, osWaitForever);

}

void pix_decode(uint32_t color, uint8_t * buff){
	uint8_t i;
	//uint32_t mask = 3;
	
	
	for(i=0;i<24;i=i+2){
			*buff = WS_decode[color&3];
			buff++;
			color>>=2;
		
	}
}

void pix_send(uint32_t color){
	char buff[12];
	
	pix_decode(color,buff);
	WS2812_spi_send(buff,12);
}

void WS2812_buff_send(uint32_t * pixels_b, uint16_t len){
	
uint16_t i;	
	char * to_WS_buff = WS2812_buff;
	
	
//data prepare
	for(i=0;i<len;i++){
		pix_decode(pixels_b[i],to_WS_buff); 
		to_WS_buff+=12;
	}
		WS2812_spi_send(WS2812_buff,WS2812_LENGTH*12);
}

//#endif

