/**
 * @file FOC_Math.c
 * @brief Mathematical operations for Field Oriented Control (FOC) and signal processing.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "FOC_Math.h"

/* Coordinate Transformation */

/**
 * @brief Transforms stator currents from the stationary alpha-beta frame to the rotating d-q frame.
 * @param  current_str Pointer to the motor current parameter structure.
 * @date 2026-06-02
 */
void Park_transfor(MOTOR_CURRENT_PARAM_T *current_str)       
{ 
    /* Extract trigonometric functions to avoid redundant calculations (Crucial for performance) */
    _iq sin_theta = _IQsinPU(current_str->Eletheta);
    _iq cos_theta = _IQcosPU(current_str->Eletheta);

    /* Park Transformation Matrix */
    current_str->I_q = -_IQmpy(current_str->I_Alpha, sin_theta) + _IQmpy(current_str->I_Beta, cos_theta);
    current_str->I_d = _IQmpy(current_str->I_Alpha, cos_theta) + _IQmpy(current_str->I_Beta, sin_theta);
}

/**
 * @brief Transforms stator voltages from the rotating d-q frame to the stationary alpha-beta frame.
 * @param  current_str Pointer to the motor current/voltage parameter structure.
 * @date 2026-06-02
 */
void RevPark_transfor(MOTOR_CURRENT_PARAM_T *current_str)
{
    /* Extract trigonometric functions */
    _iq sin_theta = _IQsinPU(current_str->Eletheta);
    _iq cos_theta = _IQcosPU(current_str->Eletheta);

    /* Inverse Park Transformation Matrix */
    current_str->U_Alpha = _IQmpy(current_str->U_d, cos_theta) - _IQmpy(current_str->U_q, sin_theta);
    current_str->U_Beta = _IQmpy(current_str->U_d, sin_theta) + _IQmpy(current_str->U_q, cos_theta);	
}

/**
 * @brief Transforms stator currents from the three-phase a-b-c frame to the two-phase alpha-beta frame.
 * @param  current_str Pointer to the motor current parameter structure.
 * @date 2026-06-02
 */
void Clark_transfor(MOTOR_CURRENT_PARAM_T *current_str) 
{
    /* 
     * Extract common subexpressions (Ib + Ic) and (Ib - Ic) to reduce multiplications.
     * Mathematically: I_alpha = 2/3 * [Ia - 1/2*(Ib+Ic)], I_beta = sqrt(3)/3 * (Ib-Ic).
     */
    _iq Ibc_sum = current_str->Ib + current_str->Ic;
    _iq Ibc_diff = current_str->Ib - current_str->Ic;

    current_str->I_Alpha = _IQmpy(DIV_2_3, current_str->Ia - _IQmpy(DIV_1_2, Ibc_sum));
    current_str->I_Beta = _IQmpy(Sqrt_3_div_3, Ibc_diff);
}

/**
 * @brief Transforms stator voltages from the two-phase alpha-beta frame to the three-phase a-b-c frame.
 * @param  current_str Pointer to the motor current/voltage parameter structure.
 * @date 2026-06-02
 */
void RevClark_transfor(MOTOR_CURRENT_PARAM_T *current_str) 
{
    current_str->Ua = current_str->U_Alpha;        /* Assuming Ia+Ib+Ic=0 */
    
    _iq half_Ua = _IQmpy(DIV_1_2, current_str->U_Alpha);
    _iq sqrt3_2_Ub = _IQmpy(Sqrt_3_div_2, current_str->U_Beta);

    current_str->Ub = -half_Ua + sqrt3_2_Ub;
    current_str->Uc = -half_Ua - sqrt3_2_Ub;
}

/* SVPWM Calculation */

/**
 * @brief Calculates the current space vector sector and determines the X, Y, and Z variables for SVPWM.
 * @param  current_str Pointer to the motor parameter structure (contains U_Alpha, U_Beta).
 * @param  ptr         Pointer to the FOC PWM control structure (stores sector and X,Y,Z).
 * @date 2026-06-02
 */
static void SVPWM_CalculateSector(MOTOR_CURRENT_PARAM_T *current_str, FOC_PWM_CTRL_T *ptr)
{
    int8_t A = 0, B = 0, C = 0, N = 0;
    
    /* === Key modification: Normalize to Udc/sqrt(3) base === */
    // Normalization factor: 1 / (Udc / sqrt(3)) = sqrt(3) / Udc
    _iq inv_Udc_sqrt3 = _IQdiv(_IQ(1.0), MC_Modulate_Ref);
    // Normalized alpha-beta voltages (per-unit, m ∈ [0, 1])
    _iq U_Alpha_pu = _IQmpy(current_str->U_Alpha, inv_Udc_sqrt3);
    _iq U_Beta_pu  = _IQmpy(current_str->U_Beta,  inv_Udc_sqrt3);

    /* Extract common subexpressions */
    _iq T1 = _IQmpy(U_Alpha_pu, Sqrt_3_div_2);   /* U_Alpha_pu * sqrt(3)/2 */
    _iq T2 = _IQmpy(U_Beta_pu,  DIV_1_2);        /* U_Beta_pu * 1/2 */

    /* Sector judgment variables */
    _iq VA = U_Beta_pu;
    _iq VB = T1 - T2;
    _iq VC = -T1 - T2;

    ptr->X = U_Beta_pu;
    ptr->Y = T1 + T2;
    ptr->Z = -T1 + T2;

    if(VA > _IQ(0.0f)) A = 1; else A = 0;
    if(VB > _IQ(0.0f)) B = 1; else B = 0;
    if(VC > _IQ(0.0f)) C = 1; else C = 0;
    
    N = 4*C + 2*B + A;
    
    switch(N)
    {
        case 3: ptr->sector = 1; break;
        case 1: ptr->sector = 2; break;
        case 5: ptr->sector = 3; break;
        case 4: ptr->sector = 4; break;
        case 6: ptr->sector = 5; break;
        case 2: ptr->sector = 6; break;
        default: ptr->sector = 1; break; // fallback
    }   
}

/**
 * @brief Calculates the three-phase inverter PWM duty cycles based on the identified space vector sector.
 * @param  ctrl Pointer to the FOC PWM control structure.
 * @date 2026-06-02
 */
static void SVPWM_CalulateDutyCycle(FOC_PWM_CTRL_T *ctrl)
{
    /* Assign T_First and T_Second based on the sector */
    switch(ctrl->sector)
    {
        case 2: ctrl->T_First = ctrl->Z; ctrl->T_Second = ctrl->Y; break;
        case 6: ctrl->T_First = ctrl->Y; ctrl->T_Second = -ctrl->X; break;
        case 1: ctrl->T_First = -ctrl->Z; ctrl->T_Second = ctrl->X; break;
        case 4: ctrl->T_First = -ctrl->X; ctrl->T_Second = ctrl->Z; break;
        case 3: ctrl->T_First = ctrl->X; ctrl->T_Second = -ctrl->Y; break;
        case 5: ctrl->T_First = -ctrl->Y; ctrl->T_Second = -ctrl->Z; break;
    }
    
    /* Overmodulation handling */
    _iq Tsum = ctrl->T_First + ctrl->T_Second;
    if(Tsum > Ts_IQ_Cal)
    {
        // Scale down to stay within linear region
        ctrl->T_First  = _IQdiv(ctrl->T_First, Tsum);
        ctrl->T_Second = _IQdiv(ctrl->T_Second, Tsum);
        Tsum = Ts_IQ_Cal; // now T_First + T_Second == 1.0
    }
    
    ctrl->T0 = _IQmpy(Ts_IQ_Cal - Tsum, DIV_1_2);
    
    /* Calculate half-period duties for center-aligned PWM */
    ctrl->Ta = _IQmpy(ctrl->T0, DIV_1_2);
    ctrl->Tb = ctrl->Ta + _IQmpy(ctrl->T_First, DIV_1_2);
    ctrl->Tc = ctrl->Tb + _IQmpy(ctrl->T_Second, DIV_1_2);
    
    /* Map to phase duties */
    switch(ctrl->sector)
    {
        case 2: ctrl->a_phase_duty = ctrl->Tb; ctrl->b_phase_duty = ctrl->Ta; ctrl->c_phase_duty = ctrl->Tc; break;
        case 6: ctrl->a_phase_duty = ctrl->Ta; ctrl->b_phase_duty = ctrl->Tc; ctrl->c_phase_duty = ctrl->Tb; break;
        case 1: ctrl->a_phase_duty = ctrl->Ta; ctrl->b_phase_duty = ctrl->Tb; ctrl->c_phase_duty = ctrl->Tc; break;
        case 4: ctrl->a_phase_duty = ctrl->Tc; ctrl->b_phase_duty = ctrl->Tb; ctrl->c_phase_duty = ctrl->Ta; break;
        case 3: ctrl->a_phase_duty = ctrl->Tc; ctrl->b_phase_duty = ctrl->Ta; ctrl->c_phase_duty = ctrl->Tb; break;
        case 5: ctrl->a_phase_duty = ctrl->Tb; ctrl->b_phase_duty = ctrl->Tc; ctrl->c_phase_duty = ctrl->Ta; break;
    }
}

/**
 * @brief Executes the complete Space Vector PWM (SVPWM) generation using the sector-based method.
 * @param  current_str Pointer to the motor parameter structure.
 * @param  ptr         Pointer to the FOC PWM control structure.
 * @date 2026-06-02
 */
void SVPWM_Sector_Ctrl(MOTOR_CURRENT_PARAM_T *current_str, FOC_PWM_CTRL_T *ptr)
{
    SVPWM_CalculateSector(current_str, ptr);      /* Sector calculation and judgment */
    SVPWM_CalulateDutyCycle(ptr);                 /* Duty cycle calculation for the corresponding sector */
}

/**
 * @brief Generates PWM duty cycles using Sinusoidal PWM with third-harmonic (zero-sequence) injection.
 * @param  current_str Pointer to the motor parameter structure (contains Ua, Ub, Uc).
 * @param  ptr         Pointer to the FOC PWM control structure.
 * @date 2026-06-02
 */
void SPWM_Inject_Harmonic_Ctrl(MOTOR_CURRENT_PARAM_T *current_str, FOC_PWM_CTRL_T *ptr)
{
    RevClark_transfor(current_str);

    /* Calculate max and min of three-phase voltages */
    ptr->Vabc_max = current_str->Ua;
    if (current_str->Ub > ptr->Vabc_max) ptr->Vabc_max = current_str->Ub;
    if (current_str->Uc > ptr->Vabc_max) ptr->Vabc_max = current_str->Uc;
    
    ptr->Vabc_min = current_str->Ua;
    if (current_str->Ub < ptr->Vabc_min) ptr->Vabc_min = current_str->Ub;
    if (current_str->Uc < ptr->Vabc_min) ptr->Vabc_min = current_str->Uc;    

    /* Calculate zero-sequence component (Equivalent to SVPWM) */
    ptr->Vabc_Zero = -_IQmpy((ptr->Vabc_max + ptr->Vabc_min), _IQ(0.5));

    /* Inject zero-sequence into SPWM */
    ptr->Vabc_Inject[0] = current_str->Ua + ptr->Vabc_Zero;
    ptr->Vabc_Inject[1] = current_str->Ub + ptr->Vabc_Zero;
    ptr->Vabc_Inject[2] = current_str->Uc + ptr->Vabc_Zero;

    /* Convert to duty cycles [0.0, 1.0] */
    ptr->a_phase_duty = _IQmpy(_IQ(0.5), _IQ(1.0) - ptr->Vabc_Inject[0]);
    ptr->b_phase_duty = _IQmpy(_IQ(0.5), _IQ(1.0) - ptr->Vabc_Inject[1]);
    ptr->c_phase_duty = _IQmpy(_IQ(0.5), _IQ(1.0) - ptr->Vabc_Inject[2]);
}

/* PID Calculation */

/**
 * @brief Initializes the parameters and internal states of the generic PI controller structure.
 * @param  Ipid    Pointer to the PI controller structure.
 * @param  Kp      Proportional gain.
 * @param  Ki      Integral gain.
 * @param  SumMAX  Maximum limit for the integral sum (anti-windup).
 * @param  PI_outMax Maximum output limit of the PI controller.
 * @date 2026-06-02
 */
void MC_PI_Init(MC_PI_CONTROLLER_T *Ipid, _iq Kp, _iq Ki, _iq SumMAX, _iq PI_outMax)
{
    Ipid->Kp = Kp;
    Ipid->Ki = Ki;
    Ipid->errSumMax = SumMAX;
    Ipid->outMax = PI_outMax;

    Ipid->erro = _IQ(0.0);
    Ipid->erro_last = _IQ(0.0);
    Ipid->i_sum = _IQ(0.0);
    Ipid->now = _IQ(0.0);
    Ipid->out = _IQ(0.0);
    Ipid->target = _IQ(0.0);
}

/**
 * @brief Executes the PI controller calculation step, supporting both parallel and series configurations.
 * @param  PI_Control Pointer to the PI controller structure.
 * @param  target     Target reference value (per-unit).
 * @date 2026-06-02
 */
void MC_PI_Calculate(MC_PI_CONTROLLER_T *PI_Control, _iq target)
{
    PI_Control->target = target;
    
    /* Calculate error */
    PI_Control->erro = PI_Control->target - PI_Control->now;
    
#if (PI_CONTROLLER_TYPE == 2)
    /* ========== Parallel PI Controller ========== */
    PI_Control->i_sum += _IQmpy(PI_Control->Ki, PI_Control->erro);
    
#elif (PI_CONTROLLER_TYPE == 1)
    /* ========== Series PI Controller ========== */
    _iq Ka_out = _IQmpy(PI_Control->Kp, PI_Control->erro);
    PI_Control->i_sum += _IQmpy(Ka_out, PI_Control->Ki);
#endif
    
    /* Integral anti-windup */
    if (PI_Control->i_sum > PI_Control->errSumMax) {
        PI_Control->i_sum = PI_Control->errSumMax;
    } else if (PI_Control->i_sum < -PI_Control->errSumMax) {
        PI_Control->i_sum = -PI_Control->errSumMax;
    }
    
#if (PI_CONTROLLER_TYPE == 2)
    PI_Control->out = _IQmpy(PI_Control->Kp, PI_Control->erro) + PI_Control->i_sum;
#elif (PI_CONTROLLER_TYPE == 1)
    PI_Control->out = Ka_out + PI_Control->i_sum;
#endif
    
    /* Output saturation */
    if (PI_Control->out > PI_Control->outMax) {
        PI_Control->out = PI_Control->outMax;
    } else if (PI_Control->out < -PI_Control->outMax) {
        PI_Control->out = -PI_Control->outMax;
    }
}

/* First-Order Low-Pass Filter (LPF1) */

/**
 * @brief Initializes the first-order low-pass filter (LPF1) parameters based on the sampling period and cutoff frequency.
 * @param handle Pointer to the LPF1 handle structure.
 * @param Ts     Sampling period (seconds).
 * @param Fc     Cutoff frequency (Hz).
 */
void MC_LPF1_Init(MC_LPF1_T *handle, _iq Ts, _iq Fc)
{
    _iq Tf;
    handle->Ts = Ts;
    handle->Fc = Fc;

    Tf = _IQdiv(DIV_PI_2, handle->Fc);

    handle->Alpha = _IQdiv(handle->Ts, (handle->Ts + Tf));

    handle->Input  = 0.0f;
    handle->Output = 0.0f;
}

/**
 * @brief Executes the first-order low-pass filter calculation and returns the filtered output.
 * @param handle Pointer to the LPF1 handle structure.
 * @param input  Current input value.
 * @return _iq   Filtered output value.
 */
_iq MC_LPF1_Run(MC_LPF1_T *handle, _iq input)
{
    handle->Input = input;
    handle->Output += _IQmpy(handle->Alpha, (handle->Input - handle->Output));

    return handle->Output;
}

/* Digital Signal Processing */

/**
 * @brief Computes the recursive Radix-2 Real Fast Fourier Transform (FFT) using a decimation-in-time algorithm.
 * @param  v   Pointer to the input/output complex data array.
 * @param  n   Number of points in the FFT (must be a power of 2).
 * @param  tmp Pointer to the temporary buffer for ping-pong operation.
 * @date 2026-06-02
 */
void rfft(complex *v, int n, complex *tmp)
{
    if(n > 1) {
        int k, m;
        complex z, w, *vo, *ve;
        ve = tmp; 
        vo = tmp + n/2;
        
        /* Odd-even decomposition */
        for(k = 0; k < n/2; k++) {
            ve[k] = v[2*k];
            vo[k] = v[2*k+1];
        }
        
        /* Recursive FFT + ping-pong buffer */
        rfft(ve, n/2, v);
        rfft(vo, n/2, v);
        
        /* Butterfly operation */
        for(m = 0; m < n/2; m++) {
            w.Re = _IQcos(_IQdiv(_IQmpy(PI_2, _IQ(m)), _IQ(n)));
            w.Im = -_IQsin(_IQdiv(_IQmpy(PI_2, _IQ(m)), _IQ(n)));  // Negative sign: DFT definition
            z.Re = _IQmpy(w.Re, vo[m].Re) - _IQmpy(w.Im, vo[m].Im);
            z.Im = _IQmpy(w.Re, vo[m].Im) + _IQmpy(w.Im, vo[m].Re);
            
            v[m].Re     = ve[m].Re + z.Re;
            v[m].Im     = ve[m].Im + z.Im;
            v[m+n/2].Re = ve[m].Re - z.Re;
            v[m+n/2].Im = ve[m].Im - z.Im;
        }
    }
}

/**
 * @brief Extracts and normalizes the magnitude of a specific target frequency from the FFT result array.
 * @param  fft_result  Pointer to the FFT result array (bit-reversed order).
 * @param  target_freq Target frequency in Hz.
 * @return _iq         Normalized magnitude at the target frequency.
 * @date 2026-06-02
 */
_iq GetFreqMagnitude(complex *fft_result, int target_freq)
{   
    /* 1. Calculate FFT index for the target frequency (rounded) */
    int index = (int)(_IQtoF(_IQ(target_freq/FFT_FREQ_RES)) + 0.5f);
    
    /* 2. Calculate magnitude: |X(k)| = sqrt(Re^2 + Im^2) / N * 2 (for real signals) */
    _iq Mag = _IQmpy(_IQdiv(_IQsqrt(_IQmpy(fft_result[index].Re, fft_result[index].Re) + _IQmpy(fft_result[index].Im, fft_result[index].Im)), _IQ(FFT_N)), _IQ(2.0));
    if(index == 0) Mag = _IQdiv(Mag, _IQ(2.0));        /* DC component adjustment */
    
    return Mag;
}

/**
 * @brief Calculates the magnitude spectrum from FFT results and prints the frequency and amplitude for each bin via UART.
 * @param  x Pointer to the complex FFT result array.
 * @param  n Number of points in the FFT.
 * @date 2026-06-02
 */
void Asm_Mag(complex *x, int n) 
{
    int i;
    _iq Mag;
    for(i = 0; i < n/2; i++)
    {
        Mag = _IQmpy(_IQdiv(_IQsqrt(_IQmpy(x[i].Re, x[i].Re) + _IQmpy(x[i].Im, x[i].Im)), _IQ(FFT_N)), _IQ(2.0));
        if(i == 0) Mag = _IQdiv(Mag, _IQ(2.0));        

        uart_printf("Bin %d: %.3f Hz, Magnitude: %.3f \n", i, _IQtoF(_IQmpy(_IQ(i), _IQ(FFT_FREQ_RES))), _IQtoF(Mag));
    }
}

