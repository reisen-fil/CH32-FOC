#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include "headfile.h"

// #define I2C_Hardware

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
uint8_t drv_I2C_WriteOneByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint8_t drv_I2C_ReadOneByte(uint8_t devAddr, uint8_t regAddr);

uint8_t drv_SoftI2C_ReadTwoBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *data);


#endif

