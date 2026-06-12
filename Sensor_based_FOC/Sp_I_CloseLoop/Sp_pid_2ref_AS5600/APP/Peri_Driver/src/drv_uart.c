/**
 * @file drv_uart.c
 * @brief USART driver implementation using DMA for efficient data transmission and reception with a circular buffer for frame parsing.
 * @author reisen-fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "drv_uart.h"

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
static void usart3_cb_init(USART_CIRCULAR_BUFFER_T *cb);
static void usart3_cb_put_frame(USART_CIRCULAR_BUFFER_T *cb, uint16_t start_offset, uint16_t length);

/*********************************************************************
 * @fn      drv_USART_DMA_init
 *
 * @brief   Configures the USART peripheral, associated DMA channels for TX/RX, GPIO pins, and NVIC for IDLE line interrupt handling.
 *
 * @param   USART - Pointer to the USART peripheral base address.
 * @param   baudrate - USART communication baud rate.
 * @param   usart_phandle - Pointer to the USART DMA system handle.
 *
 * @return  None
 */
static void drv_USART_DMA_init(USART_TypeDef *USART, uint32_t baudrate, USART_DMA_SYSTEM_T *usart_phandle)
{
    DMA_InitTypeDef DMA_InitStructure = {0};    
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* Configure DMA1 Channel3 for USART RX (Circular mode) */
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart_phandle->dma_rx_buffer;     /* Memory destination address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;          /* Peripheral to Memory */
    DMA_InitStructure.DMA_BufferSize = USART3_DMA_BUFFER_SIZE;  /* Buffer size */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;    
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    /* Configure DMA1 Channel2 for USART TX (Normal mode, dynamically configured later) */
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = 0;       /* Dynamically set during transmission */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;      /* Memory to Peripheral */
    DMA_InitStructure.DMA_BufferSize = 0;           /* Dynamically set during transmission */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;    
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel3, ENABLE);     /* Enable DMA1 Channel3 for continuous reception */

    /* Configure GPIO for USART3 TX (PB10) and RX (PB11) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART parameters */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART, &USART_InitStructure);

    /* Configure NVIC for USART3 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    

    USART_ITConfig(USART, USART_IT_IDLE, ENABLE);      /* Enable USART IDLE line interrupt for frame detection */

    USART_Cmd(USART, ENABLE);

    /* Initialize the circular buffer and reset DMA cursor */
    usart3_cb_init(&usart_phandle->RX_Circular_Buffer);
    usart_phandle->dma_rx_old_pos = 0; 

    /* Enable USART DMA requests for RX and TX */
    USART_DMACmd(USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART, USART_DMAReq_Tx, ENABLE);  
}

/**
 * @brief Top-level initialization function for the USART subsystem, configuring USART3 with DMA and a specific baud rate.
 */
void Drv_USART_Init(void)
{
    drv_USART_DMA_init(USART3, 1382400, &mc_usart_dma_handle);
}

/**
 * @brief Transmits data via DMA in polling mode, blocking until the transfer is complete.
 * @param buf Pointer to the data buffer to be transmitted.
 * @param len Number of bytes to transmit.
 */
void USART_DMA_Send(uint8_t *buf, uint16_t len)
{
    // uint8_t test_1;
    
    /* 1. Configure DMA transfer parameters */
    DMA1_Channel2->MADDR = (uint32_t)buf;           /* Memory source address */
    DMA1_Channel2->CNTR = len;                       /* Number of data items to transfer */
    
    /* 2. Clear Transfer Complete (TC) flag */
    DMA1->INTFCR |= DMA1_IT_TC2;
    
    /* 3. Enable the DMA channel */
    DMA1_Channel2->CFGR |= DMA_CFGR3_EN;
    
    /* 4. Polling wait for transfer completion */
    while(DMA1_Channel2->CNTR != 0)
    {
        /* Timeout protection can be added here */
        // test_1 = 1;
    }
    
    /* 5. Wait for TC flag to be set (ensure physical transfer is truly complete) */
    while(!(DMA1->INTFR & DMA1_IT_TC2))
    {
        // test_1 = 0;        
    };
    
    /* 6. Clear the flag and disable the DMA channel */
    DMA1->INTFCR |= DMA1_IT_TC2;
    DMA1_Channel2->CFGR &= ~DMA_CFGR3_EN;
}

/**
 * @brief Handles the USART IDLE interrupt by calculating the received frame length, handling circular buffer wrap-around, and storing the frame metadata.
 * @param usart_phandle Pointer to the USART DMA system handle.
 */
void usart3_dma_handle(USART_DMA_SYSTEM_T *usart_phandle)
{
    /* 1. Read SR and DR registers to clear the IDLE interrupt flag */
    /* Must read SR first, then DR */
    uint16_t temp = USART3->STATR;
    temp = USART3->DATAR;
    
    /* 2. Calculate the length of received data */
    /* In circular mode: received data = total buffer size - current DMA remaining counter */       
    uint16_t remaining = DMA_GetCurrDataCounter(DMA1_Channel3);
    uint16_t new_pos = USART3_DMA_BUFFER_SIZE - remaining;

    /* Calculate frame length and handle wrap-around */
    if (new_pos != usart_phandle->dma_rx_old_pos) {
        uint16_t length;
        if (new_pos > usart_phandle->dma_rx_old_pos) {
            length = new_pos - usart_phandle->dma_rx_old_pos;
        } else {
            length = USART3_DMA_BUFFER_SIZE - usart_phandle->dma_rx_old_pos + new_pos;
        }

        /* Limit maximum frame length to prevent overflow */
        // if (length > USART3_MAX_FRAME_SIZE) {
        //     length = USART3_MAX_FRAME_SIZE;
        // }

        /* Store frame metadata into the circular buffer */
        usart3_cb_put_frame(&usart_phandle->RX_Circular_Buffer, usart_phandle->dma_rx_old_pos, length);
    }

    /* Update the DMA cursor */
    usart_phandle->dma_rx_old_pos = new_pos;
}

/**
 * @brief Interrupt Service Routine (ISR) for USART3, triggered by the IDLE line to process incoming DMA data frames.
 */
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        usart3_dma_handle(&mc_usart_dma_handle);
    }
}

/* Circular buffer storage implementation */

/**
 * @brief Initializes the circular buffer control block.
 * @param cb Pointer to the circular buffer control block.
 * @details Clears the frame count, read index, and write index to prepare for receiving new data.
 */
static void usart3_cb_init(USART_CIRCULAR_BUFFER_T *cb)
{
    cb->frame_count = 0;
    cb->write_index = 0;
    cb->read_index = 0;
}

/**
 * @brief Adds a new frame's metadata to the circular buffer.
 * @param cb Pointer to the circular buffer control block.
 * @param start_offset The starting offset of the data in the DMA buffer.
 * @param length The length of the data frame.
 * @details Stores the frame metadata (offset and length) into the circular buffer, automatically handling overflow by overwriting old frames.
 */
static void usart3_cb_put_frame(USART_CIRCULAR_BUFFER_T *cb, uint16_t start_offset, uint16_t length)
{
    /* Store metadata of the new frame */
    cb->frame_info[cb->write_index].start_offset = start_offset;
    cb->frame_info[cb->write_index].length = length;
    cb->write_index = (cb->write_index + 1) % USART3_RING_BUFFER_COUNT;
    
    /* If buffer is not full, increment frame count; if full, overwrite old frame without incrementing count */
    if (cb->frame_count < USART3_RING_BUFFER_COUNT) {
        cb->frame_count++;
    }
}

/**
 * @brief Retrieves a complete data frame from the circular buffer and copies it to the user buffer within the system handle.
 * @param usart_phandle Pointer to the USART DMA system handle.
 * @retval 1 Successfully retrieved a frame.
 * @retval 0 No data frame available.
 * @details Automatically handles DMA buffer wrap-around and copies the data into the user buffer.
 */
int USART3_GetFrame(USART_DMA_SYSTEM_T *usart_phandle)
{
    if (usart_phandle->RX_Circular_Buffer.frame_count == 0) {
        return 0; /* No available data frames */
    }

    /* Get metadata of the current frame */
    USART_RXBUFF_INFO_T *frame = &usart_phandle->RX_Circular_Buffer.frame_info[usart_phandle->RX_Circular_Buffer.read_index];
    uint16_t start = frame->start_offset;
    uint16_t length = frame->length;

    /* Handle DMA buffer wrap-around */
    if (start + length <= USART3_DMA_BUFFER_SIZE) {
        /* Data does not cross the buffer boundary, copy directly */
        memcpy(usart_phandle->RX_Buffer.rxbuf, &usart_phandle->dma_rx_buffer[start], length);
    } else {
        /* Data crosses the boundary, copy in two parts */
        uint16_t part1 = USART3_DMA_BUFFER_SIZE - start;
        memcpy(usart_phandle->RX_Buffer.rxbuf, &usart_phandle->dma_rx_buffer[start], part1);
        memcpy(usart_phandle->RX_Buffer.rxbuf + part1, usart_phandle->dma_rx_buffer, length - part1);
    }
    usart_phandle->RX_Buffer.rxlen = length;

    /* Update read index and decrement frame count */
    usart_phandle->RX_Circular_Buffer.read_index = (usart_phandle->RX_Circular_Buffer.read_index + 1) % USART3_RING_BUFFER_COUNT;
    usart_phandle->RX_Circular_Buffer.frame_count--;
    return 1;
}