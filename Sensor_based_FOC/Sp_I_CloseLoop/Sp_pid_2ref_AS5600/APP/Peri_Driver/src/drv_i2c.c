/**
 * @file drv_i2c.c
 * @brief Software and Hardware I2C bus driver implementation for sensor communication (e.g., AS5600).
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "drv_i2c.h"

/**
 * @brief Configures the SDA GPIO pin as a floating input mode for reading data from the I2C slave.
 * @date 2026-06-19
 */
static void drv_SetSDA_In(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* Floating input mode */
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);
}

/**
 * @brief Configures the SDA GPIO pin as a push-pull output mode for driving the I2C bus.
 * @date 2026-06-19
 */
static void drv_SetSDA_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; /* Push-pull output mode */
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);
}

#ifdef I2C_Hardware
/**
 * @brief Recovers the I2C bus from a deadlock state by manually clocking the SCL line without power-cycling the slave.
 * @param I2Cx Pointer to the I2C peripheral, such as I2C1 or I2C2.
 * @param GPIOx Pointer to the GPIO port where the I2C pins are located.
 * @param PinSCL SCL pin mask.
 * @param PinSDA SDA pin mask.
 * @retval 0 Bus unlocked successfully.
 * @retval 1 Bus unlock failed because SDA remains low.
 * @date 2026-06-19
 */
static uint8_t I2C_BusUnlock_Recovery(I2C_TypeDef* I2Cx, GPIO_TypeDef* GPIOx, 
                               uint16_t PinSCL, uint16_t PinSDA)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* 1. Completely reset the hardware I2C state machine and clear all residual flags */
    I2C_Cmd(I2Cx, DISABLE);
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    Tick_Delay_Us(10);  /* Wait for the reset to take effect */
    I2C_SoftwareResetCmd(I2Cx, DISABLE);
    // I2Cx->STAR1 = 0x00;  /* Clear writable flags in STAR1 */
    // I2Cx->STAR2 = 0x00;  /* Clear writable flags in STAR2 */

    /* 2. Switch SCL/SDA to GPIO push-pull output mode (take over bus control) */
    GPIO_InitStruct.GPIO_Pin  = PinSCL | PinSDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    /* 3. Ensure SDA is initially high */
    GPIO_SetBits(GPIOx, PinSDA);
    GPIO_SetBits(GPIOx, PinSCL);

    /* 4. Send 9 clock pulses (force the slave to complete the current byte transmission and release SDA) */
    for(uint8_t i = 0; i < 9; i++)
    {
        GPIO_ResetBits(GPIOx, PinSCL); /* SCL low */
        Tick_Delay_Us(5);              /* Meet the minimum timing for I2C standard mode (≥2.5μs) */
        GPIO_SetBits(GPIOx, PinSCL);   /* SCL high */
        Tick_Delay_Us(5);
    }

    /* 5. Generate a STOP condition to restore the bus to idle state */
    GPIO_ResetBits(GPIOx, PinSDA);     /* Pull SDA low first */
    Tick_Delay_Us(2);
    GPIO_SetBits(GPIOx, PinSCL);       /* Pull SCL high */
    Tick_Delay_Us(2);
    GPIO_SetBits(GPIOx, PinSDA);       /* Pull SDA high (STOP condition) */
    Tick_Delay_Us(5);                  /* Wait for the bus to stabilize */

    /* 6. Check if SDA is successfully released */
    GPIO_InitStruct.GPIO_Pin  = PinSDA;
    GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_IPU; /* Switch to push up input mode to read the logic level */
    GPIO_Init(GPIOx, &GPIO_InitStruct);
    
    if(GPIO_ReadInputDataBit(GPIOx, PinSDA) == Bit_RESET)
    {
        /* SDA is still pulled low; restore output mode to prevent bus conflicts */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOx, &GPIO_InitStruct);
        GPIO_SetBits(GPIOx, PinSDA);
        return 1; /* Unlock failed (possible multi-slave contention or hardware damage) */
    }

    /* 7. Restore SCL/SDA to I2C alternate function open-drain output */
    GPIO_InitStruct.GPIO_Pin  = PinSCL | PinSDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    /* 8. Re-initialize the hardware I2C (replace with your actual initialization function) */
    // I2C1_Init(); 
    I2C_Cmd(I2Cx, ENABLE);

    return 0; /* Unlock successful */
}

/**
 * @brief Checks whether the hardware I2C bus is locked and attempts bus recovery if needed.
 * @retval 0 I2C bus is available or recovery succeeded.
 * @retval 1 I2C bus recovery failed.
 * @date 2026-06-19
 */
uint8_t I2C_Lock_Check(void)
{
    uint32_t i2c_lock_timeout = 0x10;       /* Deadlock detection timeout */
    uint8_t i2c_lock_flag = 0;

    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
        if(i2c_lock_timeout-- == 0)
        {
            i2c_lock_flag = 1;
            break;
        }
    }

    if(i2c_lock_flag)       /* Bus deadlock recovery */
    {
        if(I2C_BusUnlock_Recovery(I2C1, GPIOB, GPIO_Pin_6, GPIO_Pin_7)) return 1;
    }
    
    // i2c_lock_flag = 0;
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return 0;
}

#endif

/**
 * @brief Initializes the I2C peripheral or software I2C GPIO pins based on the compilation macro.
 * @note For hardware I2C, it configures the pins as alternate function open-drain and handles bus deadlock recovery.
 *       For software I2C, it configures SCL as push-pull output and SDA as push-pull output (idle high).
 * @date 2026-06-19
 */
static void drv_I2C_init(void)
{
    #ifdef I2C_Hardware
        I2C_InitTypeDef I2C_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;

        // uint32_t i2c_lock_timeout = 0x1000;       /* Deadlock detection timeout */
        // uint8_t i2c_lock_flag = 0;
        
        /* Configure GPIO: PB6-SCL, PB7-SDA (Alternate function open-drain output) */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        
        /* Configure I2C parameters */  
        I2C_InitStructure.I2C_ClockSpeed = 400000;  /* 400kHz Fast Mode */
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
        I2C_InitStructure.I2C_OwnAddress1 = 0x00;   /* Master mode, own address set to 0 */
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        
        /* Initialize I2C peripheral */
        I2C_Init(I2C1, &I2C_InitStructure);

        /* Configure DMA parameters */  
        #if(Hardware_I2C_Mode == 2)
        {
            /* i2c+dma TX */
            DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DATAR;
            // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)default_tx_buffer; 
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* Memory -> peripheral */
            // DMA_InitStructure.DMA_BufferSize = 1;                 /* Send 1 byte by default (address) */
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; /* Do not increment the address when sending the address */
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_InitStructure.DMA_Priority = DMA_Priority_High;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_Init(DMA1_Channel6, &DMA_InitStructure);

            /* i2c+dma RX */
            DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DATAR;
            // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)i2c_dma_buffer;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;    /* Peripheral -> memory */
            // DMA_InitStructure.DMA_BufferSize = 2;                 /* Receive 2 bytes by default (data) */
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   /* Increment the address when receiving */
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;       
            DMA_InitStructure.DMA_Priority = DMA_Priority_High;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_Init(DMA1_Channel7, &DMA_InitStructure);            
        }
        #endif

        /* Start I2C */
        I2C_Cmd(I2C1, ENABLE);
    
    #else
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Configure SCL as push-pull output */
        GPIO_InitStructure.GPIO_Pin = SCL_PIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(SCL_PORT, &GPIO_InitStructure);

        /* Pull SCL high in the initial state */
        drv_SetSCL_High();

        /* Configure SDA as push-pull output (idle high) */
        drv_SetSDA_Out();
        drv_SetSDA_High();

    #endif
}

/**
 * @brief Top-level initialization function for the I2C bus driver.
 * @date 2026-06-19
 */
void Drv_I2C_Init(void)
{
    drv_I2C_init(); 
}


#ifdef I2C_Hardware

#else
    /**
     * @brief Generates the I2C START condition by pulling SDA low while SCL is high.
     */
    static void drv_SoftI2C_Start(void)
    {
        drv_SetSDA_Out();
        drv_SetSDA_High();
        drv_SetSCL_High();
        Soft_Delay();
        /* Pull SDA low while both SCL and SDA are high */
        drv_SetSDA_Low();
        Soft_Delay();
        drv_SetSCL_Low();
        Soft_Delay();
    }

    /**
     * @brief Generates the I2C STOP condition by pulling SDA high while SCL is high.
     */
    static void drv_SoftI2C_Stop(void)
    {
        drv_SetSDA_Out();
        drv_SetSCL_Low();
        drv_SetSDA_Low();
        Soft_Delay();
        drv_SetSCL_High();
        Soft_Delay();
        drv_SetSDA_High();
        Soft_Delay();
    }

    /**
     * @brief Waits for the ACK (acknowledge) signal from the I2C slave after sending a byte.
     * @retval 0 ACK received normally.
     * @retval 1 No ACK received (timeout).
     */
    static uint8_t drv_SoftI2C_WaitAck(void)
    {
        uint8_t ucErrTime = 0;

        drv_SetSDA_In();    /* Switch SDA to input mode to read the slave's ACK */
        drv_SetSDA_High();  /* Release SDA first */
        Soft_Delay();
        drv_SetSCL_High();
        Soft_Delay();

        while(drv_SDA_Read())
        {
            ucErrTime++;
            if(ucErrTime > 200)
            {
                drv_SoftI2C_Stop();
                return 1; /* No ACK received (timeout) */
            }
        }
        drv_SetSCL_Low();
        return 0; /* ACK received normally */
    }

    /**
     * @brief Generates an ACK (acknowledge) or NACK (not acknowledge) signal after receiving a byte.
     * @param ack 0 to send ACK, 1 to send NACK.
     */
    static void drv_SoftI2C_SendAck(uint8_t ack)
    {
        drv_SetSCL_Low();
        drv_SetSDA_Out();
        if(ack) {
            /* Send NACK */
            drv_SetSDA_High();
        } else {
            /* Send ACK */
            drv_SetSDA_Low();
        }
        Soft_Delay();
        drv_SetSCL_High();
        Soft_Delay();
        drv_SetSCL_Low();
    }

    /**
     * @brief Transmits a single byte (8 bits) over the software I2C bus, MSB first.
     * @param txd The byte data to be transmitted.
     */
    static void drv_SoftI2C_SendByte(uint8_t txd)
    {
        uint8_t i;
        drv_SetSDA_Out();
        for(i = 0; i < 8; i++)
        {
            drv_SetSCL_Low();
            Soft_Delay();
            if(txd & 0x80)
                drv_SetSDA_High();
            else
                drv_SetSDA_Low();
            txd <<= 1;
            Soft_Delay();
            drv_SetSCL_High();
            Soft_Delay();
        }
        /* Release SDA after transmitting 8 bits */
        drv_SetSCL_Low();
    }

    /**
     * @brief Receives a single byte (8 bits) from the software I2C bus and sends an ACK or NACK.
     * @param ack 0 to send ACK after reading, 1 to send NACK.
     * @return The received byte.
     */
    static uint8_t drv_SoftI2C_ReadByte(uint8_t ack)
    {
        uint8_t i, receive = 0;
        drv_SetSDA_In(); /* Switch SDA to input mode */
        for(i = 0; i < 8; i++)
        {
            drv_SetSCL_Low();
            Soft_Delay();
            drv_SetSCL_High();
            receive <<= 1;
            if(drv_SDA_Read()) {
                receive++;
            }
            Soft_Delay();
        }
        /* Send ACK or NACK based on the parameter */
        drv_SoftI2C_SendAck(ack);

        return receive;
    }    

#endif



/**
 * @brief Writes a single byte of data to a specific register of the I2C slave device.
 * @param devAddr 7-bit device address.
 * @param regAddr Target register address.
 * @param data    The byte data to be written.
 * @retval 0 Success.
 * @retval non-0 Failure (timeout or NACK at various stages).
 */
// uint8_t drv_I2C_WriteOneByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
// {
//     #ifdef I2C_Hardware

//         uint32_t timeout = 0x1000;
        
//         /* Wait for the bus to be idle */
//         while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
//         {
//             if(timeout-- == 0) return 1;
//         }
        
//         /* 1. Generate START condition */
//         I2C_GenerateSTART(I2C1, ENABLE);
//         timeout = 0x1000;
//         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
//         {
//             if(timeout-- == 0) return 2;
//         }
        
//         /* 2. Send device address (write mode) */
//         I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Transmitter);
//         timeout = 0x1000;
//         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//         {
//             if(timeout-- == 0) return 3;
//         }
        
//         /* 3. Send register address */
//         I2C_SendData(I2C1, regAddr);
//         timeout = 0x1000;
//         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//         {
//             if(timeout-- == 0) return 4;
//         }
        
//         /* 4. Send data byte */
//         I2C_SendData(I2C1, data);
//         timeout = 0x1000;
//         while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//         {
//             if(timeout-- == 0) return 5;
//         }
        
//         /* 5. Generate STOP condition */
//         I2C_GenerateSTOP(I2C1, ENABLE);
        
//         return 0;  

//     #else
//         drv_SoftI2C_Start();
//         /* Send device write address */
//         drv_SoftI2C_SendByte((devAddr << 1) | 0);
//         if(drv_SoftI2C_WaitAck()) {
//             drv_SoftI2C_Stop();
//             return 1;
//         }
//         /* Send register address */
//         drv_SoftI2C_SendByte(regAddr);
//         if(drv_SoftI2C_WaitAck()) {
//             drv_SoftI2C_Stop();
//             return 2;
//         }
//         /* Send the data byte to be written */
//         drv_SoftI2C_SendByte(data);
//         if(drv_SoftI2C_WaitAck()) {
//             drv_SoftI2C_Stop();
//             return 3;
//         }
//         drv_SoftI2C_Stop();
//         return 0;

//     #endif    
// }


/**
 * @brief Reads two consecutive bytes from a specific starting register of the I2C slave device.
 * @param devAddr 7-bit device address (e.g., 0x36 for AS5600).
 * @param regAddr Starting register address (e.g., 0x0C for angle data).
 * @param data    Pointer to the output array where data[0] is the high byte and data[1] is the low byte.
 * @retval 0 Success.
 * @retval 1 Failure (timeout or NACK).
 * @date 2026-06-19
 */
uint8_t drv_SoftI2C_ReadTwoBytes(uint8_t devAddr, uint8_t regAddr,uint8_t *pdata)
{
    #ifdef I2C_Hardware

        uint32_t timeout = 0x1000;

        /* Wait for the bus to be idle */
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
        {
            if(timeout-- == 0) return 1;
        } 

        timeout = 0x1000;        

        /* ========== Phase 1: Send register address ========== */
        /* 1. Generate START condition */
        I2C_GenerateSTART(I2C1, ENABLE);
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
        {
            if(timeout-- == 0) return 2;      
        }  

        timeout = 0x1000;
        /* 2. Send device address (write mode) */
        I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Transmitter);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) )
        {
            if(timeout-- == 0) return 3;      
        }                  
               
        #if(Hardware_I2C_Mode == 1)
        {   
            timeout = 0x1000;                 
            /* 3. Send register address */
            I2C_SendData(I2C1, regAddr);
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )
            {
                if(timeout-- == 0) return 4;      
            }            
        }
        #elif(Hardware_I2C_Mode == 2)
        {
            DMA1_Channel6->MADDR = (uint32_t)&regAddr;
            DMA1_Channel6->CNTR = 1;
            
            I2C_DMACmd(I2C1, ENABLE);      /* Allow I2C to issue requests to DMA */ 
            DMA_Cmd(DMA1_Channel6, ENABLE);

            /* 7. Wait for DMA transfer completion */
            timeout = 0x1000;         
            while(!DMA_GetFlagStatus(DMA1_FLAG_TC6))
            {
                if(timeout-- == 0) {
                    I2C_DMACmd(I2C1, DISABLE);                 
                    DMA_Cmd(DMA1_Channel6, DISABLE);
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    return 4;
                }
            }            

            /* Generate a stop signal to release the bus */
            I2C_GenerateSTOP(I2C1, ENABLE);            

            /* Clear the DMA status and disable DMA requests */
            I2C_DMACmd(I2C1, DISABLE);      /* Allow I2C to issue requests to DMA */ 
            DMA_Cmd(DMA1_Channel6, DISABLE);
            DMA_ClearFlag(DMA1_FLAG_TC6);
        }                          
        #endif

        /* ========== Phase 2: Read data ========== */
        /* 4. Generate repeated START condition */
        timeout = 0x1000;         
        I2C_GenerateSTART(I2C1, ENABLE);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) )
        {
            if(timeout-- == 0) return 5;      
        }               
        
        /* 5. Send device address (read mode) */
        timeout = 0x1000;        
        I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Receiver);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
        {
            if(timeout-- == 0) return 6;      
        }        

        #if(Hardware_I2C_Mode == 1)
        {
            timeout = 0x1000; 
            while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET )    /* Wait for the first byte to be received */
            {
                if(timeout-- == 0) return 1;      
            }                 

            /* 6. Read the first byte */
            pdata[0] = I2C_ReceiveData(I2C1);

            I2C_AcknowledgeConfig(I2C1, DISABLE);   /* Send NACK for the last byte */
            /* 7. Generate STOP condition */
            I2C_GenerateSTOP(I2C1, ENABLE);

            timeout = 0x1000;
            while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
            {
                if(timeout-- == 0) { I2C_AcknowledgeConfig(I2C1, ENABLE); return 1; }
            }                    

            pdata[1] = I2C_ReceiveData(I2C1);          
                
            I2C_AcknowledgeConfig(I2C1, ENABLE);            
        }
        #elif(Hardware_I2C_Mode == 2)
        {
            DMA1_Channel7->MADDR = (uint32_t)pdata;
            DMA1_Channel7->CNTR = 2;

            I2C_DMALastTransferCmd(I2C1,ENABLE);        

            /* 6. Enable DMA and I2C DMA requests */
            I2C_DMACmd(I2C1, ENABLE);      /* Allow I2C to issue requests to DMA */        
            DMA_Cmd(DMA1_Channel7, ENABLE);

            /* 7. Wait for DMA transfer completion */
            timeout = 0x1000;         
            while(!DMA_GetFlagStatus(DMA1_FLAG_TC7))
            {
                if(timeout-- == 0) {
                    I2C_DMACmd(I2C1, DISABLE);                 
                    DMA_Cmd(DMA1_Channel7, DISABLE);
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    return 7;
                }
            }        

            /* 8. Generate a stop signal to release the bus */
            I2C_DMALastTransferCmd(I2C1,DISABLE);           
            I2C_GenerateSTOP(I2C1, ENABLE);

            /* 9. Clear the DMA status and disable DMA requests */                                  
            DMA_Cmd(DMA1_Channel7, DISABLE);
            I2C_DMACmd(I2C1, DISABLE);
            DMA_ClearFlag(DMA1_FLAG_TC7);            
        }
        return 0;

        #endif
        
    #else
        /* 1. Generate START condition */
        drv_SoftI2C_Start();
        
        /* 2. Send device write address (R/W=0) */
        drv_SoftI2C_SendByte((devAddr << 1) | 0);
        if(drv_SoftI2C_WaitAck()) return 1;  /* Wait for ACK */
        
        /* 3. Send starting register address */
        drv_SoftI2C_SendByte(regAddr);
        if(drv_SoftI2C_WaitAck()) return 1;
        
        /* 4. Generate repeated START condition */
        drv_SoftI2C_Start();
        
        /* 5. Send device read address (R/W=1) */
        drv_SoftI2C_SendByte((devAddr << 1) | 1);
        if(drv_SoftI2C_WaitAck()) return 1;
        
        /* 6. Read the first byte and send ACK (indicating more bytes to follow) */
        pdata[0] = drv_SoftI2C_ReadByte(0);  /* 0 = ACK */
        
        /* 7. Read the second byte and send NACK (indicating end of reading) */
        pdata[1] = drv_SoftI2C_ReadByte(1);  /* 1 = NACK */
        
        /* 8. Generate STOP condition to release the bus */
        drv_SoftI2C_Stop();
        
        return 0;

    #endif
}

