/**
 * @file PMSM_Measure.c
 * @brief Automatic parameter identification (Rs, Lq) for Permanent Magnet Synchronous Motors (PMSM).
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "PMSM_Measure.h"

/**
 * @brief Initializes the PMSM parameter identification structure, resetting the state machine and clearing previously identified values.
 * @param  PMSM_phandle Pointer to the parameter identification structure.
 */
void PMSM_Parameter_Init(PARAM_IDENTIFY_T *PMSM_phandle)
{
    PMSM_phandle->state = PARAM_IDLE;
    PMSM_phandle->identification_in_progress = 0;
    PMSM_phandle->identification_completed = 0;
    PMSM_phandle->identification_error = 0;
    
    // Clear identified parameter values
    PMSM_phandle->Rs = _IQ(0.0f);
    PMSM_phandle->Ld = _IQ(0.0f);
    PMSM_phandle->Lq = _IQ(0.0f);
    PMSM_phandle->psi_f = _IQ(0.0f);		
}


/* Parameter identification execution */

/**
 * @brief Executes the automatic parameter identification process (Rs, Lq) for the PMSM using a state machine approach.
 * @param  PMSM_phandle Pointer to the parameter identification structure.
 * @param  foc_phandle  Pointer to the main FOC system structure.
 */
void PMSM_ParaIdentify_Function(PARAM_IDENTIFY_T *PMSM_phandle, MC_FOC_SYSTEM_T *foc_phandle)
{
	switch(PMSM_phandle->state)
	{
		case PARAM_IDLE:
            break;
            
		case PARAM_MEASURE_R_1:		/* Measure phase resistance (Rs) by applying DC voltage pulses to the three-phase windings */
			if(foc_phandle->current_sample.Ib <= _IQdiv(PHASE_CURRENT_MAX,_IQ(2.0)))
			{
				TIM1->CH1CVR = PWM_PeriodValue;				/* Lower bridge arm conducting */
				TIM1->CH2CVR = PWM_PeriodValue-PMSM_phandle->sample_count;	/* Upper bridge arm conducting with slowly increasing duty cycle */
				TIM1->CH3CVR = PWM_PeriodValue;				/* Lower bridge arm conducting */					

				if(PMSM_phandle->state_timer++ >= 30)		// Wait for a short time after sampling
				{
					/* Measure Ib current */
					ADC_CalcCurrent(&foc_phandle->current_sample);

					PMSM_phandle->sample_count++;
					PMSM_phandle->state_timer = 0;
				}
							
			}
			else
			{
				_iq PWM_Duty = _IQdiv(_IQ(PWM_PeriodValue-TIM1->CH2CVR),_IQ(PWM_PeriodValue));
			
				PMSM_phandle->sample_count = 0;
				PMSM_phandle->Rs_ref
							+= _IQmpy(_IQdiv(foc_phandle->current_sample.MC_Ud_Volt,foc_phandle->current_sample.Ib),_IQmpy(_IQ(0.666f),PWM_Duty));
			
				PMSM_phandle->identify_state++;
				
				foc_phandle->current_sample.Ia = _IQ(0.0f);
				foc_phandle->current_sample.Ib = _IQ(0.0f);
				foc_phandle->current_sample.Ic = _IQ(0.0f);
			
				TIM1->CH1CVR = 0;
				TIM1->CH2CVR = 0;
				TIM1->CH3CVR = 0;							
			
				if(PMSM_phandle->identify_state >= 3)
				{
						PMSM_phandle->identify_state = 0;
						PMSM_phandle->Rs = _IQdiv(PMSM_phandle->Rs_ref,_IQ(3.0f));
						PMSM_phandle->Rs_ref = _IQ(0.0f);
					
						PMSM_phandle->state = PARAM_IDLE;
				}			
			}	
			break;
            
		case PARAM_MEASURE_Lq:
			if(PMSM_phandle->steady_time_cnt < 120) 
				PMSM_phandle->steady_time_cnt++;		/* Wait for 10ms to stabilize */
			else
			{
				/* Inject high-frequency sine signal (1kHz). With a 12kHz execution rate, one period is divided into 12 steps */
				FOC_Ctrl(_IQmpy(_IQ(0.2f),SIN_TABLE_12F[PMSM_phandle->sample_count]),_IQ(0.0f),foc_phandle);		
				PMSM_phandle->sample_count++;
				if(PMSM_phandle->sample_count >= 11) PMSM_phandle->sample_count = 0;

				switch(PMSM_phandle->identify_state)		
				{
					case 0:		/* Wait for the current response to stabilize */
						PMSM_phandle->state_timer++;
						if(PMSM_phandle->state_timer >= 12000)	/* After injecting for 1.0s */
						{
							PMSM_phandle->state_timer = 0;
							PMSM_phandle->identify_state = 1;
						}
						break;
                        
					case 1:		/* Start acquiring the response current */
						ADC_CalcCurrent(&foc_phandle->current_sample);       // Calculate phase currents					
						Clark_transfor(&foc_phandle->current_sample);
						Park_transfor(&foc_phandle->current_sample);
						
						PMSM_phandle->state_timer++;
						if(PMSM_phandle->state_timer>1 && PMSM_phandle->state_timer<=129)	/* Sample 128 data points */
						{
							PMSM_phandle->i[PMSM_phandle->state_timer-2].Re = foc_phandle->current_sample.I_q;
							PMSM_phandle->i[PMSM_phandle->state_timer-2].Im = _IQ(0.0);
						}
						else if(PMSM_phandle->state_timer>129) /* After sampling, perform FFT to extract the amplitude */
						{
							rfft(PMSM_phandle->i,FFT_N,PMSM_phandle->scratch_i);
							PMSM_phandle->rfft_I_amp = GetFreqMagnitude(PMSM_phandle->i,FFT_inject_Freq);		
                            
                            /* Calculate Lq using V = L * di/dt -> L = V / (2 * pi * f * I) */
							PMSM_phandle->Lq = _IQdiv(_IQmpy(_IQmpy(foc_phandle->current_sample.MC_Ud_Volt,Sqrt_3_div_2),_IQ(0.2)),_IQmpy(PMSM_phandle->rfft_I_amp,_IQmpy(_IQ(FFT_inject_Freq),PI_2)));

							FOC_Ctrl(_IQ(0.0f),_IQ(0.0f),foc_phandle);

							PMSM_phandle->state_timer = 0;
							PMSM_phandle->identify_state = 0;
							PMSM_phandle->sample_count = 0;
							PMSM_phandle->rfft_I_amp = 0;						
							PMSM_phandle->steady_time_cnt = 0;

							PMSM_phandle->psi_f = 1; // Placeholder or flag for next step

							PMSM_phandle->state = PARAM_IDLE;
						}
						break;					
				}						
			}		
			break;
            
		case PARAM_MEASURE_PSI:
			break;
            
		default:
            break;
	}
}