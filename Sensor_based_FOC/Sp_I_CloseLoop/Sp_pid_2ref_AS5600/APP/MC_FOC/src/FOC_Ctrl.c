/**
 * @file FOC_Ctrl.c
 * @brief Field Oriented Control (FOC) algorithm implementation and state machine management.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */
 
#include "FOC_Ctrl.h"

void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/* FOC Algorithm Calc */

/**
 * @brief Converts d-q axis voltage commands into three-phase PWM duty cycles and updates the hardware timer registers.
 * @param  FOC_Uq      Q-axis voltage command (torque component).
 * @param  FOC_Ud      D-axis voltage command (flux component).
 * @param  foc_phandle Pointer to the main FOC system structure.
 * @date 2026-06-02
 */
void FOC_Ctrl(_iq FOC_Uq,_iq FOC_Ud,MC_FOC_SYSTEM_T *foc_phandle)
{
    MOTOR_CURRENT_PARAM_T *pSample = &foc_phandle->current_sample;
    FOC_PWM_CTRL_T* pPWMctrl = &foc_phandle->pwm_control;    

	pSample->U_q=FOC_Uq;		/* Q-axis torque current/voltage */
	pSample->U_d=FOC_Ud;		/* D-axis flux current/voltage */

	RevPark_transfor(pSample);			/* Inverse Park transformation to get Ualpha, Ubeta */
        
    #if(MC_PWM_M_Mode == 1)       // SVPWM Sector Method

        SVPWM_Sector_Ctrl(pSample,pPWMctrl);

        TIM1->CH1CVR = _IQmpyI32int(pPWMctrl->a_phase_duty,PWM_PeriodValue_2);		/* Duty cycle setup */
        TIM1->CH2CVR = _IQmpyI32int(pPWMctrl->b_phase_duty,PWM_PeriodValue_2);
        TIM1->CH3CVR = _IQmpyI32int(pPWMctrl->c_phase_duty,PWM_PeriodValue_2);

    #elif(MC_PWM_M_Mode == 2)       // Third-Harmonic Injection SPWM Method

        SPWM_Inject_Harmonic_Ctrl(pSample,pPWMctrl);

        TIM1->CH1CVR = _IQmpyI32int(pPWMctrl->a_phase_duty,PWM_PeriodValue);		/* Duty cycle setup */
        TIM1->CH2CVR = _IQmpyI32int(pPWMctrl->b_phase_duty,PWM_PeriodValue);
        TIM1->CH3CVR = _IQmpyI32int(pPWMctrl->c_phase_duty,PWM_PeriodValue);
 
    #endif       
}

/**
 * @brief Initializes the FOC state machine, PI controllers, encoder, and hardware peripherals for system startup.
 * @param  foc_phandle Pointer to the main FOC system structure.
 * @date 2026-06-02
 */
void MC_FOC_State_Init(MC_FOC_SYSTEM_T *foc_phandle)
{
    /* State machine initialization */
    foc_phandle->state_machine.current_state = FOC_STATE_IDLE;
    foc_phandle->state_machine.startup_timer = 0;
    foc_phandle->state_machine.stall_detect_timer = 0;
    foc_phandle->state_machine.error_counter = 0;

    TIM1->CH1CVR = 0;           // Charge Bootstrap Capacitor
    TIM1->CH2CVR = 0;
    TIM1->CH3CVR = 0;

    /* For parallel current loop PI, a tuned stable set is kp=0.02, ki=0.0105 */

    // MC_PI_Init(&foc_phandle->iq_control, _IQ(0.02), _IQ(0.0105),_IQ(1.0));
    // MC_PI_Init(&foc_phandle->id_control, _IQ(0.02), _IQ(0.0105),_IQ(1.0));     

    MC_PI_Init(&foc_phandle->id_control, _IQ(Idq_Ctrl_kp), _IQ(Idq_Ctrl_ki),MC_Ipi_Sum_Limit,MC_Ipi_Out_Limit);
    MC_PI_Init(&foc_phandle->iq_control, _IQ(Idq_Ctrl_kp), _IQ(Idq_Ctrl_ki),MC_Ipi_Sum_Limit,MC_Ipi_Out_Limit);
    MC_PI_Init(&foc_phandle->Sp_control, _IQ(Sp_Ctrl_kp), _IQ(Sp_Ctrl_ki),MC_SPpi_Sum_Limit,MC_SPpi_Out_Limit);

    MC_LPF1_Init(&foc_phandle->id_control_lpf1,MC_IdqLPF1_Ts,MC_IdqLPF1_Freq);
    MC_LPF1_Init(&foc_phandle->iq_control_lpf1,MC_IdqLPF1_Ts,MC_IdqLPF1_Freq);
    MC_LPF1_Init(&foc_phandle->Sp_control_lpf1,MC_SpLPF1_Ts,MC_SpLPF1_Freq);
    // MC_BTF_2_SetCoeff_Init(&foc_phandle->Sp_control_btf2,MC_SpBTF2_Freq,MC_SpBTF2_Ts);
    EncoderAlign_Init(&mc_encoder_handle);     // AS5600 Initialization 
    PMSM_Parameter_Init(&mc_pmsm_param_identify_handle);   // Motor parameter identification initialization  
     
    ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC); // Enable ADC Injected Conversion Complete Interrupt (JEOC)
    ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE);    

    Tick_Delay_Ms(100);    
}

/* Get the initial offset of the operational amplifier */

/**
 * @brief Samples and records the initial zero-offset voltages of the current sensing operational amplifiers.
 * @param  current_sample Pointer to the motor current parameter structure.
 * @date 2026-06-02
 */
void ADC_GetInitialOpAmpBias(MOTOR_CURRENT_PARAM_T *current_sample)
{
    for (uint8_t i = 0; i < 2; i++) {
        current_sample->opamp_volt_bias[i] = current_sample->adc_inject_value[i];
    }
}


/* Calculate actual phase currents */

/**
 * @brief Calculates the actual per-unit three-phase stator currents from raw ADC values by compensating for the initial offset.
 * @param  current_sample Pointer to the motor current parameter structure.
 * @date 2026-06-02
 */
void ADC_CalcCurrent(MOTOR_CURRENT_PARAM_T *current_sample)
{
    int16_t Volt_bias[3];
    for (uint8_t i = 0; i < 2; i++)
        Volt_bias[i] = current_sample->adc_inject_value[i] - current_sample->opamp_volt_bias[i];

    current_sample->Ia = -U_AMP_TO_CURRENT(Volt_bias[0]);         /* Phase current per-unit (-1 -> 1) */
    current_sample->Ib = -U_AMP_TO_CURRENT(Volt_bias[1]);

    current_sample->Ic = -(current_sample->Ia + current_sample->Ib);          
   
}

/**
 * @brief Implements the core FOC state machine logic, handling state transitions for startup, calibration, closed-loop control, and fault recovery.
 * @param  foc_phandle     Pointer to the main FOC system structure.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @date 2026-06-02
 */
static void FOC_Model_Callback(MC_FOC_SYSTEM_T *foc_phandle,ENCODER_PARAM_T *encoder_pHandle)
{
    MOTOR_CURRENT_PARAM_T *pSample = &foc_phandle->current_sample;
    FOC_STATE_MACHINE_T *pState = &foc_phandle->state_machine;
    MC_PI_CONTROLLER_T *pIqCtrl = &foc_phandle->iq_control;
    MC_PI_CONTROLLER_T *pIdCtrl = &foc_phandle->id_control;
    MC_PI_CONTROLLER_T *pSpCtrl = &foc_phandle->Sp_control;
    // MC_LPF1_T *pIdLpf1Ctrl = &foc_phandle->id_control_lpf1;
    // MC_LPF1_T *pIqLpf1Ctrl = &foc_phandle->iq_control_lpf1;
    MC_LPF1_T *pSpLpf1Ctrl = &foc_phandle->Sp_control_lpf1;
    // MC_LPF1_T *pBtf2Ctrl = &foc_phandle->Sp_control_btf2;

    // Read data from the two injected channels
    pSample->adc_inject_value[0] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    pSample->adc_inject_value[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);

    /* State machine main loop */
    switch (pState->current_state) {
        
        case FOC_STATE_IDLE:
        {
            /* Clear Flag */
            pState->startup_timer = 0;         
            pState->stall_detect_timer = 0;
            pState->error_counter = 0;

            encoder_pHandle->isCalibrated = 0;    // Clear Calibrated_Flag
            
            /* LED initialization */
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);
            GPIO_ResetBits(GPIOC,GPIO_Pin_15);

            pState->current_state = FOC_STATE_VOLTAGE_CHECK;    // Update State
            
        }
        break;

        case FOC_STATE_VOLTAGE_CHECK:
        {
            /* DC bus voltage range check (For 12V system, supply voltage must be at least 11.6V) */
            pSample->MC_Ud_Volt = _IQmpy(_IQ(pSample->adc_dma_value[1]), _IQ(MC_Udc_ADC_TransRate));

            // Fixed-point calculation introduces error --> Actual is ~12.08V, calculated as 11.8V
            if (pSample->MC_Ud_Volt >= _IQ(MC_Udc_UNDER_LIMIT)) {
                /* Voltage normal, transition to startup state */
                pState->startup_timer = 0;
                pState->current_state = FOC_STATE_CALIB;
            } 
            else{
                /* DC bus under-voltage, enter fault state */
                pState->current_state = FOC_STATE_FAULT;
            }
        }
        break;

        case FOC_STATE_CALIB:        
        {
            /* Calibration state: Op-amp offset acquisition and encoder calibration */
            // Op-amp offset acquisition
            if(!pSample->adc_bias_ready) {                  
                ADC_GetInitialOpAmpBias(pSample);
                pSample->adc_bias_ready = 1;
            }
            // Encoder calibration             
            else
            {
                if(encoder_pHandle->isCalibrated == 1)            /* After successful calibration */
                {
                    pState->startup_timer++;
                    if(pState->startup_timer >= 8000)     /* Stabilization time after calibration */
                    {
                        /* Calibration phase completed, transition to closed-loop operation */
                        pState->startup_timer = 0;
                        pState->current_state = FOC_STATE_STOP;   /* Enter stop state machine, waiting for control commands */

                        GPIO_SetBits(GPIOC,GPIO_Pin_14);    /* Normal operation indicator LED */                    
                    }
                }
                else EncoderAlign_Calibrate(encoder_pHandle,foc_phandle);       /* Currently calibrating */
            }

        }
        break;

        case FOC_STATE_STOP:
        {
            /* Check startup conditions */
            if (Target_Sp_test >= _IQ(0.05)) {  /* Startup request exists */
                pState->current_state = FOC_STATE_CLOSED_LOOP;
            }
        }
        break;
        
        case FOC_STATE_CLOSED_LOOP:
        {    
            ADC_CalcCurrent(pSample);       // Phase current calculation

            EncoderAlign_UpdateAngle(encoder_pHandle);

            if(I_Ctrl_cnt >= 5)         /* Speed loop execution part */
            {
                int16_t Sp_Period_Counter = Get_TIM2_Counter(); 
                EncoderAlign_UpdateSpeed(encoder_pHandle,Sp_Period_Counter);
                pSpCtrl->now = MC_LPF1_Run(pSpLpf1Ctrl,EncoderAlign_GetSpeed(encoder_pHandle));
                // pSpCtrl->now = MC_BTF_2_Proc(pBtf2Ctrl,EncoderAlign_GetSpeed(encoder_pHandle));                 
                // pSpCtrl->now = EncoderAlign_GetSpeed(encoder_pHandle);                

                MC_PI_Calculate(pSpCtrl,Target_Sp_test);
                I_Ctrl_cnt = 0;                
            }

            I_Ctrl_cnt++;            
            
            // pSample->Eletheta_current = EncoderAlign_GetElectricalAngle(encoder_pHandle);

            pSample->Eletheta = EncoderAlign_GetElectricalAngle(encoder_pHandle);
            pSample->theta = EncoderAlign_GetMechanicalAngle(encoder_pHandle);
            // pSample->Now_Encoder = EncoderAlign_GetNowEncoder(encoder_pHandle);
            
            Clark_transfor(pSample);
            Park_transfor(pSample);               

            pIqCtrl->now = pSample->I_q;
            pIdCtrl->now = pSample->I_d;
            // pIdCtrl->now = MC_LPF1_Run(pIdLpf1Ctrl,pSample->I_d);
            // pIqCtrl->now = MC_LPF1_Run(pIqLpf1Ctrl,pSample->I_q);            

            // PID calculation, target d-axis current is 0
            MC_PI_Calculate(&foc_phandle->iq_control, pSpCtrl->out);  // Output limit 0.7 (modulation ratio)
            MC_PI_Calculate(&foc_phandle->id_control, _IQ(0.0));

            // Use PID outputs for FOC control
            FOC_Ctrl(pIqCtrl->out,pIdCtrl->out,foc_phandle);            
            
            // FOC_CNT++;

            // if(FOC_CNT >= 2)
            // {
            //     FOC_CNT = 0;                 
            // }            

            
            // FOC_Ctrl_cnt = TIM2->CNT;

            /* Stall detection */
            if (pSample->I_q > _IQ(0.7)) {         
                if (++pState->stall_detect_timer > 500) {
                    pState->current_state = FOC_STATE_STALL_DETECT;
                }
            } else {
                pState->stall_detect_timer = 0;
            }
        
        }
        break;

        case FOC_STATE_STALL_DETECT:
        case FOC_STATE_FAULT:
        {
            /* Fault state handling */
            
            // Stop voltage output            
            FOC_Ctrl(_IQ(0.0),_IQ(0.0),foc_phandle);            
            GPIO_SetBits(GPIOC,GPIO_Pin_15);      /* Turn on red LED to indicate error */
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);                 
            
            /* Fault recovery logic (optional delayed recovery) */
            if (pState->error_counter++ > 12000) {
                /* Fault cleared, return to idle state */
                pState->error_counter = 0;
                pState->current_state = FOC_STATE_IDLE;
            }
        }
        break;
    }
}


/* Main interrupt for FOC execution */

/**
 * @brief Interrupt Service Routine (ISR) for the ADC injected conversion complete event, triggering the periodic execution of the FOC algorithm.
 * @date 2026-06-02
 */
void ADC1_2_IRQHandler(void) {
    // Check if ADC injected conversion complete interrupt occurred //ADC1->STATR & (ADC_IT_JEOC>>8)
    if (ADC_GetITStatus(ADC2, ADC_IT_JEOC) == SET) {

            FOC_Model_Callback(&mc_foc_handle,&mc_encoder_handle);
      
        }

        ADC2->STATR &= ~(0x04);

        // if(test_cnt == 0) 
        // {
        //     GPIO_SetBits(GPIOC,GPIO_Pin_14);
        //     test_cnt = 1;
        // }
        // else 
        // {
        //     GPIO_ResetBits(GPIOC,GPIO_Pin_14);
        //     test_cnt = 0;            
        // }          

}