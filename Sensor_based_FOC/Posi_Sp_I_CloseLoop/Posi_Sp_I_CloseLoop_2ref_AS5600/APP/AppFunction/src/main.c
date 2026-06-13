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
            Target_Po_test = _IQ(string_to_float((char *)mc_usart_dma_handle.RX_Buffer.rxbuf));                         
            memset(mc_usart_dma_handle.RX_Buffer.rxbuf,0,mc_usart_dma_handle.RX_Buffer.rxlen);
        }

        MC_Param_Printf();

        // if(PMSM_ParaIdentify.psi_f == 1)
        // {
        //     Asm_Mag(i,FFT_N);
        //     Asm_Mag(v,FFT_N);            
        //     // printf("Ld:%f,%f\n",_IQtoF(PMSM_ParaIdentify.Ld),_IQtoF(MC_Ud));
        //     PMSM_ParaIdentify.psi_f = 0;
        // }        
        
        // printf("Rs:%f,%f,%f\n",_IQtoF(PMSM_ParaIdentify.Rs),_IQtoF(PMSM_ParaIdentify.Ld),_IQtoF(PMSM_ParaIdentify.Lq)); 

    }
}




