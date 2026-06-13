#ifndef __EXTERNAL_FUNCTION_H
#define __EXTERNAL_FUNCTION_H

#include "headfile.h"

void MC_Param_Printf(void);

/* delay */
void Tick_Delay_Us(uint32_t n);
void Tick_Delay_Ms(uint32_t n);
void Soft_Delay(void);

/* Key */
uint8_t Key_GetNum(void);

/* knot_Ctrl */
void PotentiometerCtrl_Init(POTENTIOMETER_CTRL_T *pHandle);
_iq PotentiometerCtrl_Update(POTENTIOMETER_CTRL_T *pHandle,MC_FOC_SYSTEM_T *foc_phandle);

void External_Function_Init(void);

int uart_printf(const char *fmt, ...);
float string_to_float(char *str);

#endif  
