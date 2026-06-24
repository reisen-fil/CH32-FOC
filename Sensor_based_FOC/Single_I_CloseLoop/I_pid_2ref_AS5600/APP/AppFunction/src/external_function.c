/**
 * @file external_function.c
 * @brief Implementation of external utilities including UART printf, delays, key scanning, and potentiometer control for FOC parameter tuning.
 * @author reisen-fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-03-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "external_function.h"

/* Host PC parameter monitoring */

/**
 * @brief Prints the real-time FOC parameters (currents, angles, PWM duties, targets, etc.) to the UART for host PC monitoring.
 * @date 2026-06-02
 */
void MC_Param_Printf(void)
{
    uart_printf("I:%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
                    mc_encoder_handle.nowEncoder,
                    _IQtoF(mc_foc_handle.current_sample.Eletheta),        
                    _IQtoF(Target_iq_test),        
                    _IQtoF(mc_foc_handle.current_sample.I_q),
                    _IQtoF(mc_foc_handle.current_sample.I_d),
                    _IQtoF(mc_foc_handle.current_sample.Ia),
                    _IQtoF(mc_foc_handle.current_sample.Ib),
                    _IQtoF(mc_foc_handle.current_sample.Ic),
                    _IQtoF(mc_foc_handle.iq_control.out),
                    _IQtoF(mc_foc_handle.id_control.out),                    
                    _IQtoF(mc_pmsm_param_identify_handle.Rs),
                    _IQtoF(mc_pmsm_param_identify_handle.Ld),
                    _IQtoF(mc_pmsm_param_identify_handle.Lq),                    
                    TIM1->CH1CVR,
                    TIM1->CH2CVR,
                    TIM1->CH3CVR);        
}

/* SysTick-based delay functions */

/**
 * @brief Generates a precise microsecond delay using the SysTick timer.
 * @param n Number of microseconds to delay.
 * @date 2026-06-02
 */
void Tick_Delay_Us(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * (SystemCoreClock / 1000000);

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4);
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->SR &= ~(1 << 0);       /* Clear compare flag */    
    SysTick->CTLR &= ~(1 << 0);
}

/**
 * @brief Generates a precise millisecond delay using the SysTick timer.
 * @param n Number of milliseconds to delay.
 * @date 2026-06-02
 */
void Tick_Delay_Ms(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * (SystemCoreClock / 1000);

    SysTick->CMP = i;
    SysTick->CTLR |= (1 << 4);
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->SR &= ~(1 << 0);       /* Clear compare flag */
    SysTick->CTLR &= ~(1 << 0);
}

/**
 * @brief Generates a short software delay using NOP instructions, typically used for bit-banging protocols like I2C to adjust the clock rate.
 * @date 2026-06-02
 */
void Soft_Delay(void)
{
    volatile uint16_t i = 2;  /* Increase or decrease to adjust the I2C speed (delay for N * 5 main clock cycles) */
    while(i--) {
        __asm("nop");       
    }
}

/* Push button scanning */

/**
 * @brief Initializes the GPIO pins for the push buttons as input with internal pull-up resistors.
 * @date 2026-06-02
 */
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief Scans the push buttons with software debouncing and returns the corresponding key code.
 * @return uint8_t The key code (1 for Key1, 2 for Key2, 0 if no key is pressed).
 * @date 2026-06-02
 */
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0)
    {
        Tick_Delay_Ms(20);
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0);  /* Wait until the button is released (blocking) */
        Tick_Delay_Ms(20);                                           /* Debounce delay */
        KeyNum = 1;                                             /* Assign the corresponding key code */
    }
    
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0)
    {
        Tick_Delay_Ms(20);
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0);
        Tick_Delay_Ms(20);
        KeyNum = 2;
    }

    return KeyNum;      /* Return the corresponding key code */
}

/* LED control */

/**
 * @brief Initializes the GPIO pins for the LEDs as push-pull outputs.
 * @date 2026-06-02
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* Rotary control knob (Potentiometer) */

/**
 * @brief Initializes the potentiometer control module, resetting the raw ADC values and reference outputs to zero.
 * @param pHandle Pointer to the potentiometer control structure.
 * @date 2026-06-02
 */
void PotentiometerCtrl_Init(POTENTIOMETER_CTRL_T *pHandle)
{
    pHandle->adcRaw = 0;
    pHandle->UqRefRaw = _IQ(0.0f);
    // pHandle->speedRefFiltered = _IQ(0.0f);
    // pHandle->maxSpeedRef = _IQ(1.0f); /* Default maximum is 1.0 */
    pHandle->enable = 1;
}

/**
 * @brief Reads the potentiometer ADC value, maps it to a normalized Q15 reference, and updates the control handle.
 * @param pHandle Pointer to the potentiometer control structure.
 * @param foc_phandle Pointer to the main FOC system structure (used to access ADC DMA values).
 * @return _iq The normalized Q15 reference value, or -1.0 if the module is disabled.
 * @note It is recommended to call this function periodically in the PWM ISR or main loop (e.g., at 1kHz).
 * @date 2026-06-02
 */
_iq PotentiometerCtrl_Update(POTENTIOMETER_CTRL_T *pHandle, MC_FOC_SYSTEM_T *foc_phandle)
{
    if (pHandle->enable == 0)
    {
        // pHandle->speedRefFiltered = _IQ(0.0f);
        return _IQ(-1.0);
    }

    /* 1. Read the raw ADC value (assuming 12-bit ADC, 0-4095) */
    pHandle->adcRaw = foc_phandle->current_sample.adc_dma_value[0];
    
    /* 2. Map to 0.0-1.0 (Q15 format) */
    /* Formula: Ref = ADC / 4095.0 */
    /* Optimize using _IQmpyI32: IQ number * integer = IQ number */
    /* _IQ(1.0/4095.0) is pre-calculated to avoid division in every cycle */
    pHandle->UqRefRaw = _IQmpyI32(_IQ(Potent_ADC_TransRate), (long)pHandle->adcRaw);

    return pHandle->UqRefRaw;
}

/**
 * @brief Top-level initialization for external peripherals, including LEDs and the potentiometer control module.
 * @date 2026-06-02
 */
void External_Function_Init(void)
{
    // KEY_Init();
    LED_Init();

    PotentiometerCtrl_Init(&mc_potentiometer_ctrl_handle);    
}

/**
 * @brief Converts the numeric payload in a VOFA command packet to an IQ fixed-point value.
 * @param Vofa_data_bag Pointer to the received VOFA data packet buffer.
 * @return _iq Converted signed fixed-point value parsed from the packet payload.
 * @date 2026-06-19
 */
_iq RxPacket_Data_Handle(uint8_t* Vofa_data_bag,uint8_t Format_Offset)
{
  _iq Data = _IQ(0.0);
  _iq decimal_weight = _IQ(0.1);
  uint8_t dot_Flag = 0;
  int8_t minus_Flag = 1;

  for(uint8_t i = 0; i < 10; i++)
  {
    uint8_t ch = Vofa_data_bag[i + Format_Offset];

    if(ch == 0x2D)
    {
      minus_Flag = -1;
      continue;
    }

    if(ch == 0x2E)
    {
      dot_Flag = 1;
      continue;
    }

    if((ch < 0x30) || (ch > 0x39))
    {
      continue;
    }

    if(dot_Flag == 0)
    {
      Data = _IQmpy(Data, _IQ(10.0)) + _IQ((long)(ch - 0x30));
    }
    else
    {
      Data += _IQmpy(_IQ((long)(ch - 0x30)), decimal_weight);
      decimal_weight = _IQmpy(decimal_weight, _IQ(0.1));
    }
  }

  if(minus_Flag < 0)
  {
    Data = -Data;
  }

//   return _IQtoF(Data);
  return Data;
}

/**
 * @brief Parses the received string buffer to extract and set the target speed based on the configured communication mode.
 * @param Buffer Pointer to the received null-terminated string buffer containing the command and data.
 * @date 2026-06-24
 */
void CtrlModeSet_Handle(uint8_t* Buffer)
{
    #if(MC_CM_Select == 1)
    {
        if(!strncmp((char*)Buffer,"Target_Iq = ",12)) Target_iq_test = RxPacket_Data_Handle(Buffer,12);
    }
    #elif(MC_CM_Select == 2)
    {
        if(!strncmp((char*)Buffer,"Iq=",3)) Target_iq_test = RxPacket_Data_Handle(Buffer,3);
    }
    #endif
}

/**
 * @brief Processes incoming control commands from UART or CAN to update the FOC state machine and parameter identification states.
 * @param foc_phandle Pointer to the FOC system handle structure.
 * @param PMSM_phandle Pointer to the PMSM parameter identification handle structure.
 * @param uart_rx_buffer Pointer to the UART receive buffer structure (active when MC_CM_Select == 1).
 * @param can_rx_buffer Pointer to the CAN receive data buffer (active when MC_CM_Select == 2).
 * @note The active communication interface (UART or CAN) is determined by the MC_CM_Select compilation macro.
 * @date 2026-06-24
 */
void MC_CTRL_RX_Handle(MC_FOC_SYSTEM_T *foc_phandle,PARAM_IDENTIFY_T *PMSM_phandle,USART_RX_READ_T *uart_rx_buffer,uint8_t *can_rx_buffer)
{    
    FOC_STATE_MACHINE_T *pFocState = &foc_phandle->state_machine; 

    if(pFocState->current_state >= FOC_STATE_STANDBY)
    {
        #if(MC_CM_Select == 1)
        {        
            switch(pFocState->vofa_ctrlfoc_state)       
            {
                case DEVICE_IDLE:
                    if(!strcmp((char*)uart_rx_buffer->rxbuf,"CURRENT_CTRL_MODE"))
                    {
                        pFocState->vofa_ctrlfoc_state = FOC_CURRENT_CTRL_MODE;                    
                        pFocState->current_state = FOC_STATE_CLOSED_LOOP;       /* Speed closed-loop control state setting */                    
                    }
                    else if(!strcmp((char*)uart_rx_buffer->rxbuf,"PARAM_IDENTI_MODE"))
                    {
                        pFocState->vofa_ctrlfoc_state = PARAM_IDENTIFY_MODE;

                        pFocState->current_state = FOC_STATE_PARAM_MEASURE;
                        PMSM_phandle->state = PARAM_IDLE;  
                    } 
                    break;                                     
                case FOC_CURRENT_CTRL_MODE:        
                    CtrlModeSet_Handle(uart_rx_buffer->rxbuf);      /* Enter host Current control mode */
                    if(!strcmp((char*)uart_rx_buffer->rxbuf,"DEVICE_IDLE"))
                    {
                        pFocState->vofa_ctrlfoc_state = DEVICE_IDLE;       /* Exit this control mode and return to the initial state */
                        pFocState->current_state = FOC_STATE_STANDBY;       /* Current closed-loop control state setting */                             
                    }
                    break;
                case PARAM_IDENTIFY_MODE:
                    if(!strcmp((char*)uart_rx_buffer->rxbuf,"RS_IDENTIFY")) PMSM_phandle->state = PARAM_MEASURE_R_1;
                    else if(!strcmp((char*)uart_rx_buffer->rxbuf,"LD_IDENTIFY")) PMSM_phandle->state = PARAM_MEASURE_Ld;
                    else if(!strcmp((char*)uart_rx_buffer->rxbuf,"LQ_IDENTIFY")) PMSM_phandle->state = PARAM_MEASURE_Lq;
                    else if(!strcmp((char*)uart_rx_buffer->rxbuf,"DEVICE_IDLE"))
                    {
                        pFocState->vofa_ctrlfoc_state = DEVICE_IDLE;       /* Exit this control mode and return to the initial state */
                        pFocState->current_state = FOC_STATE_STANDBY;       /* Current closed-loop control state setting */                    
                    }                                
                    break;
                default:break;
            }
            memset(uart_rx_buffer->rxbuf,0,uart_rx_buffer->rxlen);
        }
        #elif(MC_CM_Select == 2)
        {
            switch(pFocState->vofa_ctrlfoc_state)       
            {                
                case DEVICE_IDLE:
                    if(!strcmp((char*)can_rx_buffer,"IQ_CTRL"))
                    {
                        pFocState->vofa_ctrlfoc_state = FOC_CURRENT_CTRL_MODE;                    
                        pFocState->current_state = FOC_STATE_CLOSED_LOOP;       /* Current closed-loop control state setting */                    
                    } 
                    break;                                     
                case FOC_CURRENT_CTRL_MODE:        
                    CtrlModeSet_Handle(can_rx_buffer);      /* Enter host current control mode */
                    if(!strcmp((char*)can_rx_buffer,"RETURN"))
                    {
                        pFocState->vofa_ctrlfoc_state = DEVICE_IDLE;       /* Exit this control mode and return to the initial state */
                        pFocState->current_state = FOC_STATE_STANDBY;       /* Current closed-loop control state setting */                             
                    }
                    break;
                default:break;                
            }
            mc_can_handle.rx_finish_flag = 0;             
        }
        #endif
    }
}

/* Lightweight UART print implementation (without standard printf) */

/**
 * @brief Flushes the UART transmit buffer by sending its contents via DMA.
 * @param usart_phandle Pointer to the USART DMA system handle.
 * @date 2026-06-02
 */
static void uart_flush(USART_DMA_SYSTEM_T *usart_phandle)
{
    if (usart_phandle->tx_out_pos > 0) {
        USART_DMA_Send(usart_phandle->tx_out_buf, usart_phandle->tx_out_pos);
        usart_phandle->tx_out_pos = 0;
    }
}

/**
 * @brief Appends a single character to the UART transmit buffer, flushing it automatically if full.
 * @param ch The character to append.
 * @param usart_phandle Pointer to the USART DMA system handle.
 * @date 2026-06-02
 */
static void uart_putchar(char ch, USART_DMA_SYSTEM_T *usart_phandle)
{
    if (usart_phandle->tx_out_pos >= UART_OUT_BUF_SIZE) {
        uart_flush(usart_phandle);
    }
    usart_phandle->tx_out_buf[usart_phandle->tx_out_pos++] = (uint8_t)ch;
}

/**
 * @brief Appends a string of a specified length to the UART transmit buffer.
 * @param str Pointer to the string.
 * @param len Length of the string.
 * @param usart_phandle Pointer to the USART DMA system handle.
 * @date 2026-06-02
 */
static void uart_putstr(const char *str, int len, USART_DMA_SYSTEM_T *usart_phandle)
{
    while (len-- > 0) {
        uart_putchar(*str++, usart_phandle);
    }
}

/**
 * @brief Converts a signed 32-bit integer to a null-terminated string in the specified base.
 * @param buf Buffer to store the resulting string.
 * @param val The integer value to convert.
 * @param base The numerical base (e.g., 10, 16).
 * @param uppercase Use uppercase letters for hex (if base is 16).
 * @return int The length of the resulting string.
 * @date 2026-06-02
 */
static int itoa_custom(char *buf, int32_t val, uint8_t base, uint8_t uppercase)
{
    const char *digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";
    char temp[32];
    int i = 0;
    uint32_t uval;
    int negative = 0;

    if (val < 0 && base == 10) {
        negative = 1;
        uval = -val;
    } else {
        uval = (uint32_t)val;
    }

    if (uval == 0) {
        buf[0] = '0';
        return 1;
    }

    while (uval) {
        temp[i++] = digits[uval % base];
        uval /= base;
    }

    int pos = 0;
    if (negative) {
        buf[pos++] = '-';
    }

    while (i > 0) {
        buf[pos++] = temp[--i];
    }

    return pos;
}

/**
 * @brief Converts a floating-point number to a string with a specified decimal precision.
 * @param buf Buffer to store the resulting string.
 * @param val The floating-point value to convert.
 * @param prec Number of decimal places (max 6).
 * @return int The length of the resulting string.
 * @date 2026-06-02
 */
static int ftoa_custom(char *buf, float val, uint8_t prec)
{
    int pos = 0;

    if (prec > 6) prec = 6;

    if (val < 0) {
        buf[pos++] = '-';
        val = -val;
    }

    /* Integer part */
    int32_t int_part = (int32_t)val;
    pos += itoa_custom(&buf[pos], int_part, 10, 0);

    buf[pos++] = '.';

    /* Fractional part */
    float frac = val - int_part;
    for (uint8_t i = 0; i < prec; i++) {
        frac *= 10;
        int digit = (int)frac;
        buf[pos++] = '0' + digit;
        frac -= digit;
    }

    return pos;
}

/**
 * @brief Converts an unsigned 32-bit integer to a null-terminated string in the specified base.
 * @param buf Buffer to store the resulting string.
 * @param val The unsigned integer value to convert.
 * @param base The numerical base (e.g., 10, 16).
 * @param uppercase Use uppercase letters for hex (if base is 16).
 * @return int The length of the resulting string.
 * @date 2026-06-02
 */
static int utoa_custom(char *buf, uint32_t val, uint8_t base, uint8_t uppercase)
{
    const char *digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";
    char temp[32];
    int i = 0;

    if (val == 0) {
        buf[0] = '0';
        return 1;
    }

    while (val) {
        temp[i++] = digits[val % base];
        val /= base;
    }

    int pos = 0;
    while (i > 0) {
        buf[pos++] = temp[--i];
    }

    return pos;
}

/**
 * @brief Core formatting engine that parses the format string and arguments, converting them to strings and pushing them to the UART buffer.
 * @param fmt The format string.
 * @param args The variable argument list.
 * @param usart_phandle Pointer to the USART DMA system handle.
 * @return int The total number of characters printed.
 * @date 2026-06-02
 */
static int uart_vprintf_impl(const char *fmt, va_list args, USART_DMA_SYSTEM_T *usart_phandle)
{
    int count = 0;
    int len;
    char ch;
    char *str;
    int32_t d;
    uint32_t u;
    float f;

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;

            /* Check for %% */
            if (*fmt == '%') {
                uart_putchar('%', usart_phandle);
                fmt++;
                count++;
                continue;
            }

            /* Parse width (optional) */
            int width = 0;
            char pad = ' ';
            if (*fmt == '0') {
                pad = '0';
                fmt++;
            }
            while (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
                fmt++;
            }

            /* Parse precision (%.Nf) */
            int prec = 6;
            if (*fmt == '.') {
                fmt++;
                prec = 0;
                while (*fmt >= '0' && *fmt <= '9') {
                    prec = prec * 10 + (*fmt - '0');
                    fmt++;
                }
            }

            /* Handle format specifiers */
            switch (*fmt) {
                /* Signed integer */
                case 'd':
                case 'i':
                    d = va_arg(args, int32_t);
                    len = itoa_custom(usart_phandle->printf_buf, d, 10, 0);
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* Unsigned integer */
                case 'u':
                    u = va_arg(args, uint32_t);
                    len = utoa_custom(usart_phandle->printf_buf, u, 10, 0);
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* Lowercase hexadecimal */
                case 'x':
                    u = va_arg(args, uint32_t);
                    len = utoa_custom(usart_phandle->printf_buf, u, 16, 0);
                    
                    /* Pad width */
                    if (width > len) {
                        for (int i = 0; i < width - len; i++) {
                            uart_putchar(pad, usart_phandle);
                            count++;
                        }
                    }
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* Uppercase hexadecimal */
                case 'X':
                    u = va_arg(args, uint32_t);
                    len = utoa_custom(usart_phandle->printf_buf, u, 16, 1);
                    
                    if (width > len) {
                        for (int i = 0; i < width - len; i++) {
                            uart_putchar(pad, usart_phandle);
                            count++;
                        }
                    }
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* Octal */
                case 'o':
                    u = va_arg(args, uint32_t);
                    len = utoa_custom(usart_phandle->printf_buf, u, 8, 0);
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* Floating-point number */
                case 'f':
                case 'F':
                    f = (float)va_arg(args, double);
                    len = ftoa_custom(usart_phandle->printf_buf, f, prec);
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len;
                    break;

                /* String */
                case 's':
                    str = va_arg(args, char *);
                    if (str == NULL) {
                        uart_putstr("(null)", 6, usart_phandle);
                        count += 6;
                    } else {
                        len = strlen(str);
                        uart_putstr(str, len, usart_phandle);
                        count += len;
                    }
                    break;

                /* Character */
                case 'c':
                    ch = (char)va_arg(args, int);
                    uart_putchar(ch, usart_phandle);
                    count++;
                    break;

                /* Pointer (hexadecimal) */
                case 'p':
                    u = va_arg(args, uint32_t);
                    uart_putstr("0x", 2, usart_phandle);
                    len = utoa_custom(usart_phandle->printf_buf, u, 16, 0);
                    uart_putstr(usart_phandle->printf_buf, len, usart_phandle);
                    count += len + 2;
                    break;

                default:
                    uart_putchar('%', usart_phandle);
                    uart_putchar(*fmt, usart_phandle);
                    count += 2;
                    break;
            }

            fmt++;
        } else if (*fmt == '\n') {
            /* Automatically convert \n to \r\n */
            uart_putchar('\r', usart_phandle);
            uart_putchar('\n', usart_phandle);
            fmt++;
            count += 2;
        } else {
            uart_putchar(*fmt, usart_phandle);
            fmt++;
            count++;
        }
    }

    uart_flush(usart_phandle);
    return count;
}

/**
 * @brief Lightweight printf implementation that formats the input string and transmits it via the UART DMA interface.
 * @param fmt The format string.
 * @param ... Variable arguments.
 * @return int The total number of characters printed.
 * @date 2026-06-02
 */
int uart_printf(const char *fmt, ...)
{
    va_list args;
    int count;

    va_start(args, fmt);
    count = uart_vprintf_impl(fmt, args, &mc_usart_dma_handle);
    va_end(args);

    return count;
}

