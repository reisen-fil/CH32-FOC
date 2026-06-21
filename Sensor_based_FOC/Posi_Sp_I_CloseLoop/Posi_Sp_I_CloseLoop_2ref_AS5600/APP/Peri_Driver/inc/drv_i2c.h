#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include "headfile.h"

// #define I2C_Hardware
#ifdef I2C_Hardware
    #define Hardware_I2C_Mode        2                   /* 1:default; 2:add DMA */
#endif
/* Testing shows that when using hardware I2C at a 400 kHz clock rate to read the angle, one complete FOC execution takes about 157 us. When using software-simulated I2C (with delays implemented by assembly NOP instructions), the total time is slightly shorter than hardware I2C, about 148 us, so software simulation is used. */

#define SDA_PORT    GPIOB
#define SDA_PIN     GPIO_Pin_3
#define SCL_PORT    GPIOA
#define SCL_PIN     GPIO_Pin_15

#define drv_SDA_Read()  GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN)

#define drv_SetSCL_High()     GPIO_SetBits(SCL_PORT, SCL_PIN)
#define drv_SetSCL_Low()     GPIO_ResetBits(SCL_PORT, SCL_PIN)
#define drv_SetSDA_High()     GPIO_SetBits(SDA_PORT, SDA_PIN)
#define drv_SetSDA_Low()     GPIO_ResetBits(SDA_PORT, SDA_PIN)

void Drv_I2C_Init(void);
// uint8_t drv_I2C_WriteOneByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

#ifdef I2C_Hardware
    uint8_t I2C_Lock_Check(void);
#endif
uint8_t drv_SoftI2C_ReadTwoBytes(uint8_t devAddr, uint8_t regAddr,uint8_t *pdata);


#endif

