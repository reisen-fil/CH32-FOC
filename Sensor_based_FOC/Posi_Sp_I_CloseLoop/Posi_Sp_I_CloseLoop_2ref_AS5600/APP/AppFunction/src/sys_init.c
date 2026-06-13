/**
 * @file sys_init.c
 * @brief System-level initialization for the motor control firmware, including peripheral clock configuration, interrupt priority grouping, and module initialization sequencing.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "sys_init.h"

/* Hardware System Initialization */

/**
 * @brief Configures the system clocks and enables the peripheral clocks for GPIO, Timers, ADC, I2C, USART, and DMA.
 */
static void Drv_System_Init(void)
{
    SystemCoreClockUpdate();    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    #ifdef I2C_Hardware
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    #endif    
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);    /* ADC clock: 18MHz */

    /* Enable AFIO clock if pin remapping is required */
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief Initializes the Programmable Fast Interrupt Controller (PFIC) / NVIC and configures the interrupt priority grouping for nested interrupts.
 */
static void Drv_PFIC_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);     /* Configure interrupt priority grouping for nested interrupts (2 preemption bits, 2 sub-priority bits) */    
}

/**
 * @brief Orchestrates the initialization sequence for all low-level hardware drivers, starting with system clocks and NVIC, followed by specific peripheral drivers.
 */
static void My_DRV_Init(void)
{    
    Drv_System_Init();            /* System clock and peripheral clock configuration (RCC) */
    Drv_PFIC_Init();

    Drv_I2C_Init();
    Drv_TIM_Init();    
    Drv_USART_Init();       
    Drv_ADC_Init();         
}

/**
 * @brief Initializes the motor control hardware drivers and external user-interface peripherals (e.g., LEDs, potentiometers, keys).
 */
static void MC_Module_Init(void)
{
    My_DRV_Init();                  /* Low-level hardware drivers */
    External_Function_Init();       /* External peripherals and user interface modules */
}

/**
 * @brief Top-level system initialization function that initializes all hardware/peripheral modules and sets up the FOC state machine and motor control parameters.
 */
void MC_Sys_Init(void)
{
    MC_Module_Init();
    MC_FOC_State_Init(&mc_foc_handle);
}