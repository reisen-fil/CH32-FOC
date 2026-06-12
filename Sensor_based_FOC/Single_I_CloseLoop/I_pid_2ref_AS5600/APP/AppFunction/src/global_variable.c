/**
 * @file global_variable.c
 * @brief Definition and initialization of global variables and system handles used across the motor control firmware.
 * @author reisen-fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "global_variable.h"

/* Debug and test variables */
uint32_t FOC_Ctrl_cnt;       /* Execution time counter for FOC control loop */
uint8_t test_cnt;            /* General purpose test counter */
uint16_t AS5600_Get_cnt;     /* Execution time counter for AS5600 angle reading */
_iq Target_iq_test;          /* Target q-axis current for testing and tuning */

/* Push button and ISR communication variables */
uint8_t Get_KeyNum;          /* Captured key code from button scanning */
uint8_t main_to_isr;         /* Flag for communication between main loop and ISR */

/* Main FOC system control handle */
MC_FOC_SYSTEM_T mc_foc_handle;

/* Encoder parameter and state handle */
ENCODER_PARAM_T mc_encoder_handle;

/* PMSM parameter identification handle */
PARAM_IDENTIFY_T mc_pmsm_param_identify_handle;

/**
 * @brief Lookup table for the amplitude of the injected sine signal during parameter identification.
 * @details Contains 12 points corresponding to angles: 0°, 30°, 60°, 90°, 120°, 150°, 180°, 210°, 240°, 270°, 300°, 330°.
 */
const _iq SIN_TABLE_12F[12] = {
     _IQ(0.0000), _IQ(0.5000), _IQ(0.8660), _IQ(1.0000), _IQ(0.8660), _IQ(0.5000),
     _IQ(0.0000), _IQ(-0.5000), _IQ(-0.8660), _IQ(-1.0000), _IQ(-0.8660), _IQ(-0.5000)
};

/* Potentiometer control handle for reference input */
POTENTIOMETER_CTRL_T mc_potentiometer_ctrl_handle;

/* USART DMA communication system handle */
USART_DMA_SYSTEM_T mc_usart_dma_handle;