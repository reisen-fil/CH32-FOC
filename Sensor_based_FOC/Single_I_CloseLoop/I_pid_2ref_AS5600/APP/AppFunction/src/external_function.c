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
 */
void MC_Param_Printf(void)
{
    uart_printf("I:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
                    _IQtoF(Target_iq_test),        
                    _IQtoF(mc_foc_handle.current_sample.I_q),
                    _IQtoF(mc_foc_handle.current_sample.I_d),
                    _IQtoF(mc_foc_handle.current_sample.Ia),
                    _IQtoF(mc_foc_handle.current_sample.Ib),
                    _IQtoF(mc_foc_handle.current_sample.Ic),
                    _IQtoF(mc_foc_handle.iq_control.out),
                    _IQtoF(mc_foc_handle.id_control.out),                    
                    mc_encoder_handle.nowEncoder,
                    _IQtoF(mc_foc_handle.current_sample.Eletheta),                    
                    TIM1->CH1CVR,
                    TIM1->CH2CVR,
                    TIM1->CH3CVR);        
}

/* SysTick-based delay functions */

/**
 * @brief Generates a precise microsecond delay using the SysTick timer.
 * @param n Number of microseconds to delay.
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
 */
void External_Function_Init(void)
{
    // KEY_Init();
    LED_Init();

    PotentiometerCtrl_Init(&mc_potentiometer_ctrl_handle);    
}


/* Lightweight UART print implementation (without standard printf) */

/**
 * @brief Flushes the UART transmit buffer by sending its contents via DMA.
 * @param usart_phandle Pointer to the USART DMA system handle.
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

/**
 * @brief Parses a null-terminated string representing a decimal number and converts it to a floating-point value.
 * @param str The input string to parse.
 * @return float The converted floating-point value, or 0.0f if the input is NULL.
 */
float string_to_float(char *str) {
    if (str == NULL) {
        return 0.0f; /* Error handling, return 0 */
    }

    float result = 0.0f;
    int sign = 1; /* Sign: 1 for positive, -1 for negative */
    int i = 0;

    /* Skip leading whitespace */
    while (isspace(str[i])) {
        i++;
    }

    /* Check sign */
    if (str[i] == '-') {
        sign = -1;
        i++;
    } else if (str[i] == '+') {
        i++;
    }

    /* Parse integer part */
    while (isdigit(str[i])) {
        result = result * 10.0f + (str[i] - '0');
        i++;
    }

    /* Check decimal point */
    if (str[i] == '.') {
        i++;
        float decimal_place = 0.1f;
        while (isdigit(str[i])) {
            result += (str[i] - '0') * decimal_place;
            decimal_place *= 0.1f;
            i++;
        }
    }

    /* Apply sign */
    result *= sign;

    return result;
}