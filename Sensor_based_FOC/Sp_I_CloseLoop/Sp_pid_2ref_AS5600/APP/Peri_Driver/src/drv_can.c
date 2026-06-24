/**
 * @file drv_can.c
 * @brief CAN bus driver implementation for communication and testing.
 * @date  2026-06-24
 */
#include "drv_can.h"

uint8_t CAN_Sending_Data[CAN_Sending_Data_length] = {0x01, 0x02, 0x03, 0x04};
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/**
 * @brief   Initializes CAN communication test mode.
 *          Bps = Fpclk1 / ((tbs1 + 1 + tbs2 + 1 + 1) * brp)
 * @param   can_tsjw CAN synchronisation jump width.
 * @param   can_tbs2 CAN time quantum in bit segment 2.
 * @param   can_tbs1 CAN time quantum in bit segment 1.
 * @param   can_brp  Specifies the length of a time quantum.
 * @param   can_mode Test mode (CAN_Mode_Normal, CAN_Mode_LoopBack, CAN_Mode_Silent, CAN_Mode_Silent_LoopBack).
 * @date    2026-06-24
 */
static void drv_CAN_init(uint8_t can_tsjw, uint8_t can_tbs2, uint8_t can_tbs1, u16 can_brp, uint8_t can_mode)
{
    GPIO_InitTypeDef      GPIO_InitSturcture = {0};
    CAN_InitTypeDef       CAN_InitSturcture = {0};
    CAN_FilterInitTypeDef CAN_FilterInitSturcture = {0};
    NVIC_InitTypeDef      NVIC_InitStructure = {0};

    GPIO_InitSturcture.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitSturcture.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitSturcture.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitSturcture);

    GPIO_InitSturcture.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitSturcture.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitSturcture);

    CAN_InitSturcture.CAN_TTCM = DISABLE;  /* Time trigger mode disabled */
    CAN_InitSturcture.CAN_ABOM = DISABLE;  /* Automatic bus-off management disabled */
    CAN_InitSturcture.CAN_AWUM = DISABLE;  /* Automatic wake-up mode disabled */
    CAN_InitSturcture.CAN_NART = ENABLE;   /* No automatic retransmission (enabled) */
    CAN_InitSturcture.CAN_RFLM = DISABLE;  /* Receive FIFO locked mode disabled */
    CAN_InitSturcture.CAN_TXFP = DISABLE;  /* Transmit FIFO priority determined by identifier */
    CAN_InitSturcture.CAN_Mode = can_mode; /* Operation mode (Normal/LoopBack/Silent) */
    CAN_InitSturcture.CAN_SJW = can_tsjw;  /* Resynchronization jump width */
    CAN_InitSturcture.CAN_BS1 = can_tbs1;  /* Bit segment 1 (BS1) */
    CAN_InitSturcture.CAN_BS2 = can_tbs2;  /* Bit segment 2 (BS2) */
    CAN_InitSturcture.CAN_Prescaler = can_brp; /* Baud rate prescaler */
    CAN_Init(CAN1, &CAN_InitSturcture);

    CAN_FilterInitSturcture.CAN_FilterNumber = 0;
    CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;     /* Set to mask mode */
    CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;    /* Set to 32-bit scale mode */
    CAN_FilterInitSturcture.CAN_FilterIdHigh = 0x200 << 4;              /* Set standard frame ID in identifier register to 0x100 */
    CAN_FilterInitSturcture.CAN_FilterIdLow = 0;                        /* Configure as standard frame (IDE=0) and data frame (RTR=0), reserved bits set to 0 */
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0xE00 << 4;          /* Set standard frame in mask register to 0x700, configuring the message ID filtering range to 0x100~0x1FF */
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0;                    /* Configure as standard frame (IDE=0) and data frame (RTR=0), reserved bits set to 0 */
    CAN_FilterInitSturcture.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitSturcture.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitSturcture);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  /* CAN1 RX0 interrupt vector for CH32V203 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   /* Preemption priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          /* Sub priority */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); /* Enable FIFO0 message pending interrupt */
}

/**
 * @brief  Top-level initialization function for the CAN bus driver.
 * @note   Configures the CAN peripheral for 1Mbps baud rate in normal mode.
 * @date   2026-06-24
 */
void Drv_CAN_Init(void)
{
    drv_CAN_init(CAN_SJW_1tq, CAN_BS2_3tq, CAN_BS1_2tq, 12, CAN_Mode_Normal); /* Bps: 1Mbps */
}

/**
 * @brief  Sends a CAN standard data frame.
 * @param  StdId Standard frame ID (0x000 ~ 0x7FF).
 * @param  pData Pointer to the data buffer to be sent.
 * @param  Len   Byte length of the data (0 ~ 8).
 * @retval 0 Sending failed (no idle mailbox).
 * @retval 1 Sending request successfully pended in the mailbox.
 * @date   2026-06-24
 */
uint8_t CAN_Send_StdData(uint32_t StdId, uint8_t *pData, uint8_t Len)
{
    CanTxMsg TxMessage;
    uint8_t transmit_mailbox;

    /* Limit max length to 8 bytes to prevent out-of-bounds */
    if (Len > 8) {
        Len = 8;
    }

    /* Configure message header */
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_Id_Standard;   /* Standard frame */
    TxMessage.RTR = CAN_RTR_Data;      /* Data frame */
    TxMessage.DLC = Len;               /* Data length */

    /* Copy data */
    for (uint8_t i = 0; i < Len; i++) {
        TxMessage.Data[i] = pData[i];
    }

    /* Request transmission */
    transmit_mailbox = CAN_Transmit(CAN1, &TxMessage);

    /* Check if successfully pended in transmit mailbox */
    if (transmit_mailbox == CAN_TxStatus_NoMailBox) {
        return 0; /* All 3 transmit mailboxes full, transmission failed */
    }

    return 1; /* Success */
}

/**
 * @brief  Sends the predefined test data array via CAN.
 * @retval 0 Sending failed.
 * @retval 1 Sending request successfully pended.
 * @date   2026-06-24
 */
uint8_t CAN_Sending_Test(void)
{
    return CAN_Send_StdData(CAN_SendstdID, CAN_Sending_Data, CAN_Sending_Data_length);
}

/**
 * @brief  CAN1 FIFO0 receive interrupt service routine.
 * @note   Triggered when a message matching the filter is received in FIFO0.
 * @date   2026-06-24
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_SYSTEM_T *pCanHandle = &mc_can_handle;

    /* Check if FIFO0 message pending interrupt is triggered */
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET && !pCanHandle->rx_finish_flag) {
        /* Read message from FIFO0 */
        /* Note: CAN_Receive internally releases FIFO space automatically (clears RFOM0 bit) */
        CAN_Receive(CAN1, CAN_FIFO0, &pCanHandle->rx_message);

        if (pCanHandle->rx_message.DLC < 8) {
            for (uint8_t i = pCanHandle->rx_message.DLC; i < 8; i++) {
                pCanHandle->rx_message.Data[i] = 0;
            }
        }
        pCanHandle->rx_finish_flag = 1;
    }

    /* Clear interrupt pending bit */
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}
