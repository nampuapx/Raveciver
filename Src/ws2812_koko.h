
#include <stm32f2xx_hal.h>


int WS2812_SPI3_Init(void);
void WS2812_spi_send(char * buff,uint16_t len);
void pix_send(uint32_t color);
void WS2812_buff_send(uint32_t * pixels_b, uint16_t len);





