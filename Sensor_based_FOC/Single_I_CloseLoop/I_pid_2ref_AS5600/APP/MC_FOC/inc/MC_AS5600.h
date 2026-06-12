#ifndef __MC_AS5600_H
#define __MC_AS5600_H

#include "headfile.h"

uint16_t AS5600_GetAngle();

void EncoderAlign_Init(ENCODER_PARAM_T *encoder_pHandle);
void EncoderAlign_Calibrate(ENCODER_PARAM_T *encoder_pHandle,MC_FOC_SYSTEM_T *foc_phandle);
void EncoderAlign_UpdateAngle(ENCODER_PARAM_T *encoder_pHandle);

_iq EncoderAlign_GetElectricalAngle(ENCODER_PARAM_T *encoder_pHandle);
_iq EncoderAlign_GetMechanicalAngle(ENCODER_PARAM_T *encoder_pHandle);
_iq EncoderAlign_GetNowEncoder(ENCODER_PARAM_T *encoder_pHandle);
_iq EncoderAlign_GetSpeed(ENCODER_PARAM_T *encoder_pHandle);

#endif 
