/**
 * @file drv_adc.c
 * @brief ADC and OPAMP driver implementation for motor current and voltage sensing.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "drv_adc.h"

/**
 * @brief Initializes the internal Operational Amplifiers (OPAMP) and configures their associated GPIO pins for analog signal conditioning.
 */
static void drv_ADC_OPAMP_init(void)
{
    GPIO_InitTypeDef GPIO_IN_InitStructure;
    GPIO_InitTypeDef GPIO_OUT_InitStructure; 

    OPA_InitTypeDef opa_init;
    OPA_StructInit(&opa_init);

    /* Input pins configuration */
    GPIO_IN_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_IN_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_IN_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IN_InitStructure);

    GPIO_IN_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_IN_InitStructure);

    /* Output pins configuration (configured as analog input for ADC) */
    GPIO_OUT_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_OUT_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_OUT_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_OUT_InitStructure);

    GPIO_OUT_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_OUT_InitStructure);

    /* Configure and enable OPA1 */
    opa_init.OPA_NUM = OPA1;
    opa_init.PSEL = CHP1;
    opa_init.NSEL = CHN1;
    opa_init.Mode = OUT_IO_OUT1;
    OPA_Init(&opa_init);
    OPA_Cmd(OPA1, ENABLE);

    /* Configure and enable OPA2 */
    opa_init.OPA_NUM = OPA2;
    opa_init.PSEL = CHP1;
    opa_init.NSEL = CHN1;
    opa_init.Mode = OUT_IO_OUT1;
    OPA_Init(&opa_init);
    OPA_Cmd(OPA2, ENABLE);
}

/**
 * @brief Configures the ADC regular channels for continuous conversion and sets up DMA to transfer the data to a memory buffer.
 * @param  ADC    Pointer to the ADC peripheral base address (e.g., ADC1).
 * @param  buffer Pointer to the destination memory buffer for storing ADC conversion results.
 */
static void drv_ADCtoDMA_Regular_init(ADC_TypeDef *ADC, uint32_t *buffer)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    /* Configure GPIO pins for ADC analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure ADC peripheral */
    ADC_DeInit(ADC);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;            /* Scan mode for multi-channel sequence */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;      /* Continuous conversions (set DISABLE if triggering manually) */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /* Software trigger */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;
    ADC_Init(ADC, &ADC_InitStructure);

    /* Enable ADC and perform calibration */
    ADC_DMACmd(ADC, ENABLE);     
    ADC_Cmd(ADC, ENABLE);

    ADC_BufferCmd(ADC, DISABLE); /* Disable ADC internal buffer */
    ADC_ResetCalibration(ADC);
    while(ADC_GetResetCalibrationStatus(ADC));
    ADC_StartCalibration(ADC);
    while(ADC_GetCalibrationStatus(ADC)); 

    /* Configure DMA1 Channel1: Peripheral (ADC data) -> Memory (buffer) */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->RDATAR; /* ADC data register */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; /* Peripheral -> memory */
    DMA_InitStructure.DMA_BufferSize = 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; /* Circular mode for continuous streaming */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);   
    
    /* Configure ADC regular channels and sampling time */
    ADC_RegularChannelConfig(ADC, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    
    /* Start ADC conversion */
    ADC_SoftwareStartConvCmd(ADC, ENABLE); 
    
    Tick_Delay_Ms(10);
}

/**
 * @brief Configures the ADC injected channels for hardware-triggered conversion (via Timer TRGO) and enables the corresponding interrupt.
 * @param  ADC Pointer to the ADC peripheral base address (e.g., ADC2).
 */
static void drv_ADC_Inject_init(ADC_TypeDef *ADC)      
{
    ADC_InitTypeDef ADC_InitStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;

    ADC_DeInit(ADC);

    /* Configure ADC peripheral for injected conversion */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;      /* Independent mode */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;            /* Scan mode */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     /* Discontinuous conversion mode */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;
    ADC_Init(ADC, &ADC_InitStructure);

    /* Set the injected sequencer length to 2 channels */
    ADC_InjectedSequencerLengthConfig(ADC, 2);
    
    /* Configure the injected channels and their sampling sequence */
    ADC_InjectedChannelConfig(ADC, ADC_Channel_9, 1, ADC_SampleTime_13Cycles5);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_4, 2, ADC_SampleTime_13Cycles5);    
    
    /* Configure the trigger source for injected conversion (e.g., TIM1 TRGO event) */
    ADC_ExternalTrigInjectedConvConfig(ADC, ADC_ExternalTrigInjecConv_T1_TRGO);
    
    /* Enable external trigger for injected conversion */
    ADC_ExternalTrigInjectedConvCmd(ADC, ENABLE);

    /* Configure NVIC for ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn; /* ADC1 and ADC2 on CH32V203 share this interrupt vector */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable ADC and perform calibration */
    ADC_Cmd(ADC, ENABLE);
    ADC_BufferCmd(ADC, DISABLE); /* Disable ADC internal buffer */
    ADC_ResetCalibration(ADC);
    while(ADC_GetResetCalibrationStatus(ADC));
    ADC_StartCalibration(ADC);
    while(ADC_GetCalibrationStatus(ADC));    
}

/**
 * @brief Top-level initialization function for the ADC subsystem, sequentially initializing the OPAMP, regular channels (with DMA), and injected channels.
 */
void Drv_ADC_Init(void)
{
    drv_ADC_OPAMP_init();
    drv_ADCtoDMA_Regular_init(ADC1, (uint32_t *)mc_foc_handle.current_sample.adc_dma_value);
    drv_ADC_Inject_init(ADC2);
}

