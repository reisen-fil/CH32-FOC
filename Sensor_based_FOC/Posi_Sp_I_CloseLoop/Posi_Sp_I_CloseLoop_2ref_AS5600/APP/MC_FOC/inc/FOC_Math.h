#ifndef _FOC_MATH_H_
#define _FOC_MATH_H_

#include "headfile.h"

/* Coordinate transfor */
void Park_transfor(MOTOR_CURRENT_PARAM_T *current_str);
void RevPark_transfor(MOTOR_CURRENT_PARAM_T *current_str);
void Clark_transfor(MOTOR_CURRENT_PARAM_T *current_str);
void RevClark_transfor(MOTOR_CURRENT_PARAM_T *current_str);

/* SVPWM */
void SVPWM_Sector_Ctrl(MOTOR_CURRENT_PARAM_T *current_str,FOC_PWM_CTRL_T *ptr);
void SPWM_Inject_Harmonic_Ctrl(MOTOR_CURRENT_PARAM_T *current_str,FOC_PWM_CTRL_T *ptr);

/* pid */
void MC_PI_Init(MC_PI_CONTROLLER_T *Ipid, _iq Kp, _iq Ki, _iq SumMAX,_iq PI_outMax);
void MC_PI_Calculate(MC_PI_CONTROLLER_T *PI_Control, _iq target);
void MC_P_Calculate(MC_PI_CONTROLLER_T *PI_Control, _iq target);

/* LPF_1 */
void MC_LPF1_Init(MC_LPF1_T *handle,_iq Ts,_iq Fc);
_iq MC_LPF1_Run(MC_LPF1_T *handle,_iq input);

/* BTF_2 */
void MC_BTF_2_SetCoeff_Init(MC_BTF_2_T *filter, _iq fc, _iq Ts);
_iq MC_BTF_2_Proc(MC_BTF_2_T *filter, _iq in);

/* dsp */
void rfft(complex *v, int n, complex *tmp);
_iq GetFreqMagnitude(complex *fft_result, int target_freq);
void Asm_Mag(complex *x,int n);

#endif
