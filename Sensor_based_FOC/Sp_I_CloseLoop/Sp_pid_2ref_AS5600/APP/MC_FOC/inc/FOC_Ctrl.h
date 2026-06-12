#ifndef __FOC_CTRl_H__
#define __FOC_CTRl_H__

#include "headfile.h"

void FOC_Ctrl(_iq FOC_Uq,_iq FOC_Ud,MC_FOC_SYSTEM_T *foc_phandle);	
void MC_FOC_State_Init(MC_FOC_SYSTEM_T *foc_pHandle);

/* ADC_handle */
void ADC_GetInitialOpAmpBias(MOTOR_CURRENT_PARAM_T *current_sample);
void ADC_CalcCurrent(MOTOR_CURRENT_PARAM_T *current_sample);

#endif

