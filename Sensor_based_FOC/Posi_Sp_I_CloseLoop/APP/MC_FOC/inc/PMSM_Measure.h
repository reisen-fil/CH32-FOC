#ifndef __PMSM_MEASURE_H__
#define __PMSM_MEASURE_H__

#include "Headfile.h"
	
void PMSM_Parameter_Init(PARAM_IDENTIFY_T *PMSM_phandle);
void PMSM_ParaIdentify_Function(PARAM_IDENTIFY_T *PMSM_phandle,MC_FOC_SYSTEM_T *foc_phandle);


#endif