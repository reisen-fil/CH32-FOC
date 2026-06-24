#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "headfile.h"

#define CAN_SendstdID     0x4
#define CAN_Sending_Data_length       4

void Drv_CAN_Init(void);
uint8_t CAN_Send_StdData(uint32_t StdId, uint8_t *pData, uint8_t Len);
uint8_t CAN_Sending_Test(void);

#endif

