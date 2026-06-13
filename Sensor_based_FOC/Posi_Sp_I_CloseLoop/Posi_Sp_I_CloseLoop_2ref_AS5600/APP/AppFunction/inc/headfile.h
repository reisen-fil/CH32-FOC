#ifndef _HEADFILE_H_
#define _HEADFILE_H_

#include "ch32v20x.h"

/* C */
#include "stdio.h"
#include "math.h"
#include <stdint.h>
#include <stdarg.h>
#include "string.h"
#include <ctype.h>

/* AppFunction */
#include "global_variable.h"
#include "external_function.h"
#include "sys_init.h"
#include "IQmath_RV32.h"

/* MC_FOC */
#include "FOC_Ctrl.h"
#include "FOC_Math.h"
#include "PMSM_Measure.h"
#include "MC_AS5600.h"

/* Peri_Driver */
#include "drv_adc.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "drv_i2c.h"

#endif
