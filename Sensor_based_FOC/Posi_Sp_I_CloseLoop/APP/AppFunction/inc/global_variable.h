/**
 * @file global_variable.h
 * @brief Definitions of global variables, macros, structures, and enumerations for the motor control system.
 * @author reisen-fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __GLOBAL_VARIABLE_H
#define __GLOBAL_VARIABLE_H

#include "ch32v20x.h"
#include "IQmath_RV32.h"

/* Debug and test variables */
extern uint8_t test_cnt, I_Ctrl_cnt;
extern uint32_t FOC_Ctrl_cnt;
extern uint16_t AS5600_Get_cnt;
extern _iq Target_iq_test, Target_Sp_test,Target_Po_test;

/* Push button and ISR communication variables */
extern uint8_t Get_KeyNum, main_to_isr;

/********************************* FOC_SYSTEM *************************************/

#define PI		                        (_IQ(3.141526535))          /* Pi */
#define PI_2		                    (_IQ(6.283053070))          /* 2 * Pi */
#define Angle_PI_2		                (_IQ(360.0))                /* 360 */
#define DIV_PI_2		                (_IQ(1.0/6.283053070))      /* 1/(2 * Pi) */
#define DIV_2	                        (_IQ(1.41421356237))        /* sqrt(2) */
#define DIV_1_2	                        (_IQ(0.5))                  /* 1/2 */
#define DIV_2_3	                        (_IQ(0.66666))              /* 2/3 */
#define Sqrt_3_div_2	                (_IQ(0.86602))              /* sqrt(3)/2 */
#define Sqrt_3_div_3                    (_IQ(0.577350269))          /* sqrt(3)/3 */

#define MC_Udc	                        (12.0f)                     /* DC bus voltage (nominal) */
#define MC_Udc_UNDER_LIMIT              (11.6f)                     /* DC bus voltage under-voltage limit */      
#define ADC_TransRate                   (3.3f/4095.0f)              /* ADC actual voltage conversion rate */
#define Potent_ADC_TransRate            (1.0f/4095.0f)              /* Potentiometer ADC voltage conversion rate */
#define MC_Udc_ADC_TransRate            (5.69996f*ADC_TransRate)    /* DC bus voltage ADC conversion rate (compensated) */

#define MC_PWM_Freq                     6000.0                      /* PWM carrier frequency */
#define MC_PWM_Freq_Omega               (6.283053070*MC_PWM_Freq)   /* PWM carrier angular frequency (rad/s) */
#define PWM_PeriodValue                 1500.0                      /* PWM period value (ARR) */
#define PWM_PeriodValue_2               (PWM_PeriodValue*2)         /* Double reload value (for center-aligned PWM) */
#define MC_PWM_PrescalerValue           (uint16_t)(SystemCoreClock / (MC_PWM_Freq*PWM_PeriodValue_2)) - 1  /* PWM timer prescaler */

#define MC_Modulate_Ref                 (_IQ(MC_Udc*0.57733))       /* SVPWM modulation voltage reference */
#define MC_Ud_Calib                     (_IQmpy(MC_Modulate_Ref, _IQ(0.4))) /* Calibration d-axis voltage */
#define MC_PWM_Ts                       (1.0/MC_PWM_Freq)           /* PWM switching period (Ts) */
#define MC_SpLoop_Ts                    (5.0*MC_PWM_Ts)             /* Speed loop control period (Ts) */
#define Ts_IQ_Cal	                    (_IQ(1.0))                  /* Normalized Ts (per-unit) */

// #define SVPWM_K                         (_IQdiv(Ts_IQ_Cal,_IQ(10.0)))
#define SVPWM_K                         Ts_IQ_Cal                   /* SVPWM scaling factor */

#define PI_CONTROLLER_TYPE              1                           /* PI controller structure: 1=Series, 2=Parallel */
#define MC_PWM_M_Mode                   1                           /* FOC modulation mode: 1=SVPWM sector method, 2=Third-harmonic injection SPWM(error) */
   
/* PMSM Motor Parameters */
#define M_Rs                            8.0                         /* Phase resistance (Ohms) */
#define M_Ls                            0.00425                     /* Phase inductance (H) */
#define M_Ldq                           (1.5*M_Ls)                  /* d-axis inductance (H), in SPMSM, Lq = Ld */ 
#define M_Flux                          (0.175)                     /* Permanent magnet flux linkage (Wb) */
#define M_J                             (0.0027)                    /* Moment of inertia (kg*m^2) */

#define MOTOR_POLE_PAIRS                7.0                         /* Motor pole pairs */
#define K_Sp                            (1.5*MOTOR_POLE_PAIRS*M_Flux/M_J) /* Speed loop plant gain */

#define Ref                             0.05f			            /* Shunt resistor value (Ohms) */
#define Gain                            7.3f                        /* Current sensing op-amp gain */

#define MAXIMUM_SPEED_RPM               1000.0                      /* Maximum mechanical speed (RPM) */
#define PHASE_CURRENT_MAX               (_IQ(1.0))                  /* Maximum phase current (1A) */

// #define US_TO_MIN                       (0.0000000166f)             /* Microseconds to minutes conversion factor */               

/* Per-unit system (Normalization) base values */
#define ADC_Base                        (4095.0)                    /* ADC full-scale value */
// #define NORM_CURRENT(current_real)      (_IQ(current_real/(I_Current_Base*Ref*Gain*ADC_Base/3.3))) /* Normalize actual current to per-unit */ 
#define U_AMP_TO_CURRENT(U_diff)        (_IQ(U_diff*3.3f/Gain/Ref/ADC_Base)) /* Convert voltage difference to per-unit current */
#define Rad_Pu_TransRate                (_IQ(0.1592356))            /* Radian to per-unit conversion rate */

#define MC_I_Bandwidth                  (6.283053070*MC_PWM_Freq/25.0) /* Current loop bandwidth (rad/s) */
#define MC_Sp_Damp                      (20.0)                      /* Parallel Damp for 7.0 */

/* PI Controller Parameters Calculation */
#define Series_idq_kp                   (MC_I_Bandwidth*M_Ldq)      /* Series PI proportional gain (Current loop) */
#define Series_idq_ki                   (M_Rs*MC_PWM_Ts/M_Ldq)      /* Series PI integral gain (Current loop) */

#define Parallel_idq_kp                 (MC_I_Bandwidth*M_Ldq)      /* Parallel PI proportional gain (Current loop) */
#define Parallel_idq_ki                 (MC_I_Bandwidth*M_Rs*MC_PWM_Ts) /* Parallel PI integral gain (Current loop) */ 

#define MC_Ipi_Sum_Limit                (_IQ(MC_Udc*0.57733*0.8f))  /* Current loop integral limit (full modulation ratio is 0.8) */
#define MC_Ipi_Out_Limit                (_IQ(MC_Udc*0.57733*0.8f))  /* Current loop output limit (full modulation ratio is 0.8) */

#define Series_Sp_kb                    (Series_idq_kp/(M_Ldq*(MC_Sp_Damp*MC_Sp_Damp))) /* Series PI bandwidth factor (Speed loop) */
#define Series_Sp_ka                    (Series_Sp_kb*MC_Sp_Damp/K_Sp) /* Series PI zero factor (Speed loop) */
#define Series_Sp_ki                    (Series_Sp_kb*MC_SpLoop_Ts) /* Series PI integral gain (Speed loop) */
#define Series_Sp_kp                    Series_Sp_ka                /* Series PI proportional gain (Speed loop) */

// #define Parallel_Sp_ki                  (Parallel_idq_kp*MC_SpLoop_Ts/(K_Sp*MC_Sp_Damp))
// #define Parallel_Sp_kp                  (Parallel_idq_kp*Parallel_idq_kp)/(MC_Sp_Damp*MC_Sp_Damp*MC_Sp_Damp*K_Sp)
#define Parallel_Sp_kp                  (MC_I_Bandwidth/(K_Sp*(2.0*MC_Sp_Damp*MC_Sp_Damp+1))) /* Parallel PI proportional gain (Speed loop) */
#define Parallel_Sp_ki                  (MC_I_Bandwidth*MC_I_Bandwidth*MC_SpLoop_Ts/(K_Sp*(2.0*MC_Sp_Damp*MC_Sp_Damp+1)*(2.0*MC_Sp_Damp*MC_Sp_Damp+1))) /* Parallel PI integral gain (Speed loop) */

#define MC_SPpi_Sum_Limit               (_IQ(0.5))                 /* Speed loop integral limit */
#define MC_SPpi_Out_Limit               (_IQ(0.5))                 /* Speed loop output limit */

#define Position_Ctrl_kp                (0.08f)                     /* Position loop kp */

#define MC_POpi_Out_Limit               (_IQ(6.0))                 /* Position loop output limit */

#if(PI_CONTROLLER_TYPE == 1) 
    #define Idq_Ctrl_kp  Series_idq_kp
    #define Idq_Ctrl_ki  Series_idq_ki  
    #define Sp_Ctrl_kp   Series_Sp_kp
    #define Sp_Ctrl_ki   Series_Sp_ki
#else
    #define Idq_Ctrl_kp  Parallel_idq_kp
    #define Idq_Ctrl_ki  Parallel_idq_ki     /* Fixed typo from original code */
    #define Sp_Ctrl_kp   Parallel_Sp_kp
    #define Sp_Ctrl_ki   Parallel_Sp_ki
#endif

/* LPF_1 */
#define MC_SpLPF1_Ts                    (_IQ(MC_SpLoop_Ts))         /* First-order low-pass filter control period (Speed loop) */
#define MC_SpLPF1_Freq                  (_IQ(50.0))                 /* First-order low-pass cutoff frequency -> 50Hz (Speed loop) */
#define MC_IdqLPF1_Ts                   (_IQ(MC_PWM_Ts))            /* First-order low-pass filter control period (Current loop) */
#define MC_IdqLPF1_Freq                 (_IQ(50.0))                 /* First-order low-pass cutoff frequency -> 50Hz (Current loop) */

/* BTF_2 */
#define MC_SpBTF2_Ts                    (_IQ(MC_SpLoop_Ts))         /* Second-order Butterworth filter control period (Speed loop) */
#define MC_SpBTF2_Freq                  (_IQ(100.0))                /* Second-order Butterworth filter cutoff frequency -> 100Hz (Speed loop) */

/**************************** FOC_State_Enum ********************************/
typedef enum {
    FOC_STATE_IDLE=1,                 /* Idle state */
    FOC_STATE_VOLTAGE_CHECK,          /* Check DC bus voltage */
    FOC_STATE_CALIB,                  /* Calibration: Get OPA bias + AS5600 alignment */
    // FOC_STATE_OPEN_LOOP,            /* Open-loop run - initial startup acceleration */
    FOC_STATE_CLOSED_LOOP,            /* Closed-loop FOC control */
    FOC_STATE_STALL_DETECT,           /* Stall detection and protection */
    FOC_STATE_FAULT,                  /* Fault state */
    FOC_STATE_STOP                    /* Stop state (waiting for start command) */
} FOC_STATE_E;

/**************************** FOC_System_State ******************************/
typedef struct {
    FOC_STATE_E current_state;        /* Current state of the FOC state machine */
    // FOC_STATE_E next_state;          
    
    uint16_t startup_timer;           /* Timer for startup/calibration delays */
    // uint16_t open_loop_counter;      
    uint16_t stall_detect_timer;      /* Timer for stall detection */
    uint16_t error_counter;           /* Counter for fault state duration */
    
} FOC_STATE_MACHINE_T;

/*************************** Motor_Current_Param ****************************/
typedef struct {
	uint16_t adc_inject_value[2];		/* 2-shunt current sample from ADC injected channels */
	uint16_t adc_dma_value[2];			/* DC bus voltage and potentiometer sample from ADC DMA */
	uint16_t opamp_volt_bias[2];        /* Operational amplifier zero-offset voltage */
	uint8_t adc_bias_ready;		        /* Flag indicating op-amp bias has been acquired */
    _iq MC_Ud_Volt;                     /* DC bus voltage converted from ADC */

    _iq Ia, Ib, Ic;                     /* 3-phase stator currents */
	_iq Ua, Ub, Uc;                     /* 3-phase stator voltages */

    _iq I_Alpha, I_Beta;		        /* Stationary frame (alpha-beta) currents */
    _iq I_Alpha_H, I_Beta_H;            /* Stationary frame (alpha-beta) high frequency currents */
    _iq U_Alpha, U_Beta;                /* Stationary frame (alpha-beta) voltages */    

    _iq I_d, I_q;                       /* Rotating frame (d-q) currents */
    _iq I_d_H, I_q_H;                   /* Rotating frame (d-q) high frequency currents */    
    _iq U_d, U_q;	                    /* Rotating frame (d-q) voltages */	
    _iq Eletheta;                       /* Electrical angle */
    _iq Eletheta_current;               /* Current electrical angle (for control) */
    _iq Eletheta_esti;                  /* Estimated electrical angle (for observer) */
    _iq theta;                          /* Mechanical angle */
    _iq Now_Encoder;                    /* Current raw encoder value */

} MOTOR_CURRENT_PARAM_T;

/*************************** FOC_PWM_Control ******************************/
typedef struct {

    /* SVPWM Sector Method */
    uint8_t sector;	                /* Current space vector sector */
    _iq X, Y, Z;				        /* SVPWM algorithm intermediate variables */
    _iq T_First, T_Second;	        /* SVPWM algorithm active vector times */
    _iq T0, Ta, Tb, Tc;		 		/* SVPWM algorithm zero vector and phase times */

    /* Third-Harmonic Injection SPWM */
    _iq Vabc_max;                   /* Maximum of 3-phase voltages */
    _iq Vabc_min;                   /* Minimum of 3-phase voltages */
    
    _iq Vabc_Zero;                  /* Zero-sequence component */
    _iq Vabc_Inject[3];             /* Injected 3-phase voltages (equivalent to SVPWM) */

    _iq a_phase_duty;			    /* Phase A PWM duty cycle (register value) */
    _iq b_phase_duty;			    /* Phase B PWM duty cycle (register value) */
    _iq c_phase_duty;			    /* Phase C PWM duty cycle (register value) */    

} FOC_PWM_CTRL_T;

/***************************** Motor_Ctrl_PI *********************************/
typedef struct {
	_iq Kp, Ki;                     /* Proportional and integral gains */
	_iq target, erro, erro_last, now, i_sum, out, errSumMax, outMax; /* PI controller internal states and limits */
	
} MC_PI_CONTROLLER_T;

/***************************** First-Order Low-Pass Filter **********************************/
typedef struct
{
    _iq Ts;          /* Control period (s) */
    _iq Fc;          /* Cutoff frequency (Hz) */
    _iq Alpha;       /* Filter coefficient */

    _iq Input;       /* Current input */
    _iq Output;      /* Current output */
} MC_LPF1_T;

/************************** Second-Order Butterworth Low-Pass Filter ******************************/
typedef struct {
    _iq gain;       ///< Gain coefficient (Q16 format)
    _iq a[2];       ///< Feedback coefficients [a1, a2] (Q16 format)
    _iq b[3];       ///< Feedforward coefficients [b0, b1, b2] (Q16 format)
    _iq state[2];   ///< Filter internal state variables [state1, state2] (Q16 format)
    _iq fc;         ///< Cutoff frequency (Q16 format, unit: Hz)
    _iq Ts;         ///< Sampling period (Q16 format, unit: seconds)
} MC_BTF_2_T;

/***************************** MC_FOC_System *********************************/
typedef struct {
	FOC_STATE_MACHINE_T state_machine;      /* FOC state machine variables */
    MOTOR_CURRENT_PARAM_T current_sample;   /* Sampled and transformed motor currents/voltages */
    FOC_PWM_CTRL_T pwm_control;             /* PWM generation control variables */
    MC_PI_CONTROLLER_T iq_control;          /* Q-axis current PI controller */
    MC_PI_CONTROLLER_T id_control;          /* D-axis current PI controller */
    MC_PI_CONTROLLER_T Sp_control;          /* Speed loop PI controller */
    MC_PI_CONTROLLER_T Po_control;          /* Speed loop PI controller */    
    MC_LPF1_T id_control_lpf1;              /* D-axis current LPF1 filter */
    MC_LPF1_T iq_control_lpf1;              /* Q-axis current LPF1 filter */
    MC_LPF1_T Sp_control_lpf1;              /* Speed loop LPF1 filter */
    MC_BTF_2_T Sp_control_btf2;             /* Speed loop BTF2 filter */
	
} MC_FOC_SYSTEM_T;

extern MC_FOC_SYSTEM_T mc_foc_handle;      

/********************************* AS5600_Encoder *************************************/

#define AS5600_I2C_ADDR      0x36               /* AS5600 7-bit I2C address (W:0x6C, R:0x6D) */

#define AS5600_REG_ANGLE_H   0x0E               /* Angle high byte register */
#define AS5600_REG_ANGLE_L   0x0F               /* Angle low byte register */

#define ENCODER_RESOLUTION   4095               /* AS5600 resolution (12-bit) */

#define NORM_ENCODER(encoder_raw) (_IQdiv(_IQ(encoder_raw), _IQ(ENCODER_RESOLUTION))) /* Normalize encoder raw value to per-unit (turns) */

/************************ Encoder_Param_Handle ******************************/
typedef struct
{
    _iq ElectricalAngle_Norm;         /* Normalized electrical angle (0.0 - 1.0) */
    _iq MechanicalAngle_Norm;         /* Normalized mechanical angle (0.0 - 1.0) */
    uint16_t encoder_state_cnt;       /* Encoder state counter (used during zero alignment) */
    uint16_t rawEncoder;              /* Encoder zero-point offset (bias from alignment) */
    uint16_t nowEncoder;              /* Current raw encoder reading */
    uint16_t lastEncoder;             /* Previous raw encoder reading */
    _iq Norm_speed_min;               /* Normalized mechanical speed (RPM) */    
    uint8_t isCalibrated;             /* Calibration completion flag */
    uint8_t alignState;               /* Alignment state machine flag (zero alignment) */
	uint8_t motorDir;			      /* Motor operating direction */
    _iq dt_min;                       /* Minimum gap time */
} ENCODER_PARAM_T;

extern ENCODER_PARAM_T mc_encoder_handle;

/******************************* PMSM_Param_Identify ************************************/

/* DSP rFFT Configuration */
#define FFT_N                               128              /* 128-point FFT */
#define FFT_Fs                              12000            /* Sampling frequency: 12kHz */
#define FFT_FREQ_RES                        (FFT_Fs/FFT_N)   /* Frequency resolution = 93.75Hz */

#define FFT_inject_Freq                     1000.0           /* Frequency of the injected high-frequency signal for parameter identification */

/*************************** PMSM_Identify_State *******************************/
typedef enum {
    PARAM_IDLE,                                 /* Idle state */
    PARAM_MEASURE_R_1,                          /* Measure stator resistance */
    PARAM_MEASURE_R_2,              
    PARAM_MEASURE_Ld,                           /* Measure inductance (Ld/Lq) */
	PARAM_MEASURE_Lq,               
    PARAM_MEASURE_PSI,                          /* Measure permanent magnet flux linkage */
//    PARAM_VERIFY,                              /* Verify parameters */
    PARAM_FINISH,                               /* Identification completed */
    PARAM_ERROR                                 /* Error state */
} PARAM_IDENTIFY_STATE_T;

/******************************* PMSM_DSP_Handle *******************************/
/* DSP digital signal processing algorithm structures */
typedef struct {
    _iq Re;                                     /* Real part */
    _iq Im;                                     /* Imaginary part */
} complex;

/***************************** PMSM_Param_Handle *******************************/
typedef struct {
    /* Motor parameters */
    _iq Rs;                                     /* Stator resistance */
    _iq Ld;                                     /* d-axis inductance */
    _iq Lq;                                     /* q-axis inductance */
    _iq psi_f;                                  /* Permanent magnet flux linkage */
    
    /* Identification state machine variables */
    PARAM_IDENTIFY_STATE_T state;
    uint16_t sample_count;    
	uint8_t identify_state;
    uint16_t state_timer;
    uint8_t steady_time_cnt;                    /* Steady-state time counter */
    
    _iq angle;                                  /* Current electrical angle */
    _iq rfft_I_amp, rfft_U_amp;                 /* FFT amplitudes for current and voltage */
	_iq Rs_ref;                                 /* Resistance reference accumulator */
    
    complex v[FFT_N], scratch_v[FFT_N];         /* Voltage FFT buffers */
    complex i[FFT_N], scratch_i[FFT_N];         /* Current FFT buffers */

    /* State machine control flags */
    uint8_t identification_in_progress;
    uint8_t identification_completed;
    uint8_t identification_error;
} PARAM_IDENTIFY_T;

extern PARAM_IDENTIFY_T mc_pmsm_param_identify_handle;
extern const _iq SIN_TABLE_12F[];

/************************ Potentiometer_Ctrl_ADC ****************************/

typedef struct
{
    uint16_t adcRaw;                            /* Raw ADC value (0-4095) */
    _iq UqRefRaw;                               /* Normalized ADC value (0.0-1.0) used as Uq reference */
    // iq_t speedRefFiltered;                   /* Filtered speed reference (0-1 IQ15) */
    // iq_t maxSpeedRef;                        /* Maximum speed reference (for upper limit) */
    uint8_t enable;                             /* Enable flag */
} POTENTIOMETER_CTRL_T;

extern POTENTIOMETER_CTRL_T mc_potentiometer_ctrl_handle;

/********************************* USART+DMA_Serial *************************************/

/* Lightweight UART print buffer sizes */
#define PRINTF_BUF_SIZE 256
#define UART_OUT_BUF_SIZE 512

/* Ring Buffer Module Configuration */
#define USART3_DMA_BUFFER_SIZE      1024        /* DMA circular buffer size */
#define USART3_RING_BUFFER_COUNT    32          /* Number of frame metadata slots in the ring buffer */
#define USART3_MAX_FRAME_SIZE	   	64          /* Maximum size of a single user frame */

/******************** DMA_Buffer's Position and length ************************/
typedef struct {
    uint16_t start_offset;                      /* Starting offset of the frame in the DMA buffer */
    uint16_t length;                            /* Actual length of the frame data */
} USART_RXBUFF_INFO_T;

/************************* USART_Circular_Buffer *******************************/
typedef struct {
    uint16_t frame_count;                                   /* Number of frames pending processing */
    USART_RXBUFF_INFO_T frame_info[USART3_RING_BUFFER_COUNT]; /* Frame information queue */
    uint16_t write_index;                                   /* Ring buffer write index */
    uint16_t read_index;                                    /* Ring buffer read index */
} USART_CIRCULAR_BUFFER_T;

/************************* USART_Read_Buffer *******************************/
typedef struct {
	uint8_t rxbuf[USART3_MAX_FRAME_SIZE];       /* Data frame buffer (always starts from primary address) */
	uint16_t rxlen;                             /* Length of the current frame data */
} USART_RX_READ_T;

/************************* USART+DMA_System ********************************/
typedef struct {
    uint8_t dma_rx_buffer[USART3_DMA_BUFFER_SIZE];      /* USART+DMA RX circular buffer */
    uint16_t dma_rx_old_pos;                            /* Previous DMA cursor position */
    USART_CIRCULAR_BUFFER_T RX_Circular_Buffer;         /* Frame metadata ring buffer */
    USART_RX_READ_T RX_Buffer;                          /* User read buffer for the latest frame */

    char printf_buf[PRINTF_BUF_SIZE];                   /* USART+DMA TX printf formatting buffer */
    uint8_t tx_out_buf[UART_OUT_BUF_SIZE];              /* USART+DMA TX output circular buffer */
    uint16_t tx_out_pos;                                /* Current position in the TX output buffer */
} USART_DMA_SYSTEM_T;        		

extern USART_DMA_SYSTEM_T mc_usart_dma_handle;

#endif

