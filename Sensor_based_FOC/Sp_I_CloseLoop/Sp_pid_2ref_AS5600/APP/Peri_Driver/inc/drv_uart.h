#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "headfile.h"

void Drv_USART_Init(void);
void USART_DMA_Send(uint8_t *buf, uint16_t len);

int USART3_GetFrame(USART_DMA_SYSTEM_T *usart_phandle);


#endif


