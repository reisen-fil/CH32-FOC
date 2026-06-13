/**
 * @file drv_tim.c
 * @brief Timer driver implementation for center-aligned complementary PWM generation and system time-base counting.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "drv_tim.h"

/**
 * @brief Configures the advanced-control timer (TIM1) for center-aligned complementary PWM generation with dead-time insertion.
 * @param  TIM           Pointer to the timer peripheral base address (e.g., TIM1).
 * @param  PWM_Dead_Time The dead-time value to be inserted between complementary outputs (in timer clock cycles).
 */
static void drv_TIM1_CenterPWM_init(TIM_TypeDef *TIM, uint16_t PWM_Dead_Time)
{   
    GPIO_InitTypeDef GPIOA_InitStructure;
    GPIO_InitTypeDef GPIOB_InitStructure;    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    /* TIM1 Channels 1, 2, 3 and their complementary outputs - PA8/9/10, PB13/14/15 */
    GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  /* Alternate function push-pull output */
    GPIOA_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIOA_InitStructure);

    GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  /* Alternate function push-pull output */
    GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIOB_InitStructure);
    
    /* Time base configuration - Center-aligned mode 1 */
    TIM_TimeBaseStructure.TIM_Period = PWM_PeriodValue - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = MC_PWM_PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;  /* Center-aligned mode */
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;        
    
    TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);
    
    /* Channel 1 PWM configuration */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;        /* PWM mode 2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      /* Main output enable */
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;    /* Complementary output enable */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          /* Output polarity */
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;        /* Complementary output polarity */
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       /* Output level in idle state */
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM, &TIM_OCInitStructure);
    
    /* Channel 2 PWM configuration */
    TIM_OC2Init(TIM, &TIM_OCInitStructure);
    
    /* Channel 3 PWM configuration */
    TIM_OC3Init(TIM, &TIM_OCInitStructure);    
    
    /* Dead time and break configuration (BDTR register) */
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;     /* Off-state selection for Run mode */
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;     /* Off-state selection for Idle mode */
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = PWM_Dead_Time;             /* Dead time */
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;            /* Disable break function */
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;/* Set break polarity */
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;  /* Enable automatic output */
    TIM_BDTRConfig(TIM, &TIM_BDTRInitStructure);
    
    /* Enable preload registers to generate new reload values in the next period */
    TIM_OC1PreloadConfig(TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM, TIM_OCPreload_Enable);
    
    /* Enable shadow registers (Auto-reload preload) */
    TIM_ARRPreloadConfig(TIM, ENABLE);
    
    /* Enable MOE (Main Output Enable) for PWM */
    TIM_CtrlPWMOutputs(TIM, ENABLE);
    
    /* Select TIM1 update event as TRGO output (master mode) - Trigger source for ADC injected group sampling */
    TIM_SelectOutputTrigger(TIM, TIM_TRGOSource_Update);    

    /* Start the timer */
    TIM_Cmd(TIM, ENABLE);
}

/**
 * @brief Configures TIM2 as a basic time-base counter running at 2kHz for general system timing or debugging.
 */
static void drv_TIM2_Count_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    uint16_t PrescalerValue;

    PrescalerValue = 144 - 1;  

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period        = 10000 - 1;           
    TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    
    // /* Enable update interrupt */
    // TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);    
    // Tick_Delay_Ms(10);

    // NVIC_InitTypeDef NVIC_InitStructure;
    // NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

    /* Enable the timer */
    TIM_Cmd(TIM2, ENABLE);
}

int16_t Get_TIM2_Counter(void)
{
    int16_t Counter = TIM2->CNT;
    TIM2->CNT = 0;
    return Counter;
}

/**
 * @brief Top-level initialization function for the timer subsystem, configuring the main PWM timer and the auxiliary counter.
 */
void Drv_TIM_Init(void)
{
    drv_TIM1_CenterPWM_init(TIM1, 50);    /* Freq: 12kHz, Dead time: approx. 50ns PWM */
    drv_TIM2_Count_Init();
}