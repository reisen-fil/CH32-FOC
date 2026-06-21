/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/


#include "headfile.h"

// float Test_float1;
// uint8_t AS5600_high,AS5600_low;
// uint16_t current_angle;

/**
 * @brief Application entry point that initializes the motor-control system and runs the main service loop.
 * @return int This function does not return during normal operation.
 * @date 2026-06-02
 */
int main(void)
{
    MC_Sys_Init();   

    while(1)
    {     

        // if(test_cnt)
        // {   
        //     GPIO_SetBits(GPIOC,GPIO_Pin_15);
        //     Soft_Delay();
        //     GPIO_ResetBits(GPIOC,GPIO_Pin_15);
        //     test_cnt = 0;    
        // }
        
        if(USART3_GetFrame(&mc_usart_dma_handle))
        {
            UART_RX_Handle(&mc_foc_handle,&mc_pmsm_param_identify_handle,mc_usart_dma_handle.RX_Buffer.rxbuf);                         
            memset(mc_usart_dma_handle.RX_Buffer.rxbuf,0,mc_usart_dma_handle.RX_Buffer.rxlen);
            
        }

        MC_Param_Printf();        

        // if(mc_pmsm_param_identify_handle.psi_f == 1)
        // {
        //     MC_Param_Printf();            
        //     Asm_Mag(mc_pmsm_param_identify_handle.i,FFT_N);
        //     // Asm_Mag(v,FFT_N);            
        //     // printf("Ld:%f,%f\n",_IQtoF(PMSM_ParaIdentify.Ld),_IQtoF(MC_Ud));
        //     mc_pmsm_param_identify_handle.psi_f = 0;
        // }        
        
        // printf("Rs:%f,%f,%f\n",_IQtoF(PMSM_ParaIdentify.Rs),_IQtoF(PMSM_ParaIdentify.Ld),_IQtoF(PMSM_ParaIdentify.Lq)); 

    }
}




