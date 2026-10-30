/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file CAN_FD_APIs.c
 * @brief CAN FD API implementation for TI AM2432 Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements CAN FD communication APIs including configuration,
 * message object creation, data transmission/reception, and error handling.
 * Supports both classic CAN and CAN FD protocols with configurable bit timing.
 */

#include "../header_files/common_extern.h"
#include <string.h>
#include <kernel/dpl/SemaphoreP.h>

/* Global variables for CAN FD operation */
extern CANFD_OpenParams openParams[CONFIG_MCAN_NUM_INSTANCES];
extern CANFD_Config gCanfdConfig[CONFIG_MCAN_NUM_INSTANCES];

/* Message objects and handles for TX/RX operations */
CANFD_MessageObject txMessagObject;
CANFD_MessageObject rxMessagObject;
CANFD_MsgObjHandle  txMessageObjHandle = NULL;
CANFD_MsgObjHandle  rxMessageObjHandle = NULL;

/* Semaphore for TX completion synchronization */
static SemaphoreP_Object    gCANFDTxDoneSem;
static SemaphoreP_Object    gCANFDRxDoneSem;
uint32_t xSemaphoreTXTimeOut = 5000;  /* TX timeout in ticks */
uint32_t xSemaphoreRXTimeOut = 5000;  /* RX timeout in ticks */
volatile uint8_t gCANTxISRFlage, gCANRxISRFlage = 0;

/* Data buffers and message parameters */
uint8_t gCANReadBuff[CANFD_READ_DATA_LEN] = {0}; 
static volatile uint8_t gCANFDErrorState = 0u;  /* Error state bitmap */
uint32_t startMsgID_Tx, endMsgID_Tx, dataLen_Tx = 0;
uint32_t startMsgID_Rx, endMsgID_Rx, dataLen_Rx = 0;

/**
 * @brief Validate CAN FD data length
 * @param len Data length to validate
 * @return SystemP_SUCCESS if valid, error code otherwise
 */
int32_t CANFD_ValidateDataLen(uint32_t len)
{
    /* Use TI driver's built-in validation for CAN FD data sizes */
    return CANFD_isDataSizeValid(len);
}

/**
 * @brief Configure and initialize CAN FD peripheral
 * @param configParamsPayload Array of configuration parameters:
 *        [0]=fdMode, [1]=brsEnable, [2]=darEnable, [3-12]=bit timing params
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_Configure(uint32_t* configParamsPayload)
{
    int32_t status = SystemP_SUCCESS;
    gCANFDErrorState = 0;  /* Clear previous errors */
       
    if(configParamsPayload == NULL) return (CANFD_INVALID_PARAMS_ERR);
    
    /* Validate configuration parameters (fdMode, brsEnable, darEnable must be 0 or 1) */
    if (configParamsPayload[0] > 1U || configParamsPayload[1] > 1U || configParamsPayload[2] > 1U)
    {
        DebugP_logError("CANFD_Configure: invalid parameters (fd=%u, brs=%u, dar=%u)\r\n",
                        configParamsPayload[0], configParamsPayload[1], configParamsPayload[2]);
        return (CANFD_OUT_OF_RANGE_ERR);
    }

    /* Configure CAN FD open parameters */
    openParams[CONFIG_MCAN0].fdMode    = configParamsPayload[0]; /* FD mode enable/disable */
    openParams[CONFIG_MCAN0].brsEnable = configParamsPayload[1]; /* Bit rate switching enable */
    openParams[CONFIG_MCAN0].darEnable = configParamsPayload[2]; /* Delay after request */

    /* Configure nominal bit timing parameters */
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.nomBrp = configParamsPayload[3];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.nomPropSeg = configParamsPayload[4];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.nomPseg1 = configParamsPayload[5];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.nomPseg2 = configParamsPayload[6];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.nomSjw = configParamsPayload[7];
    
    /* Configure data bit timing parameters for FD mode */
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.dataBrp = configParamsPayload[8];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.dataPropSeg = configParamsPayload[9];    
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.dataPseg1 = configParamsPayload[10];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.dataPseg2 = configParamsPayload[11];
    gCanfdConfig[CONFIG_MCAN0].attrs->CANFDMcanBitTimingParams.dataSjw = configParamsPayload[12];

    /* Initialize and open CAN FD peripheral */
    gCanfdHandle[CONFIG_MCAN0] = NULL;
    gCanfdHandle[CONFIG_MCAN0] = CANFD_open(CONFIG_MCAN0, &openParams[CONFIG_MCAN0]);

    if (NULL == gCanfdHandle[CONFIG_MCAN0])
    {
        DebugP_logError("CANFD open failed for instance %u !!!\r\n", CONFIG_MCAN0);
        status = SystemP_FAILURE;
    }
    
    return status;
}

/**
 * @brief Create CAN FD transmit message object
 * @param id Message ID type (Standard/Extended)
 * @param startMsgID Starting message ID for transmission
 * @param endMsgID Ending message ID (for range-based filtering)
 * @param dataLen Payload data length (0-64 bytes for FD mode)
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_Transmit_MsgObjectCreate(CANFD_MCANXidType id, uint32_t startMsgID,
                                       uint32_t endMsgID, uint32_t dataLen)
{
    int32_t status = SystemP_SUCCESS;
    gCANFDErrorState = 0;  /* Clear previous errors */
    
    if (NULL == gCanfdHandle[CONFIG_MCAN0])
    {
        DebugP_logError("CANFD_Transmit_MsgObjectCreate: CANFD not opened\r\n");
        return SystemP_FAILURE;
    }

    if (CANFD_ValidateDataLen(dataLen) != SystemP_SUCCESS)
    {
        DebugP_logError("CANFD_Transmit_MsgObjectCreate: invalid dataLen=%u\r\n", dataLen);
        return CANFD_INVALID_DATA_LEN_ERR;
    }
    
    /* Initialize and configure TX message object */
    (void)memset(&txMessagObject, 0, sizeof(txMessagObject));
    txMessagObject.direction  = CANFD_Direction_TX;
    txMessagObject.msgIdType  = id;             /* Standard or Extended ID */
    txMessagObject.startMsgId = startMsgID;     /* Message ID for transmission */
    txMessagObject.endMsgId   = endMsgID;       /* End ID for range filtering */
    txMessagObject.dataLength = dataLen;        /* Payload size (FD up to 64 bytes) */
    txMessagObject.txMemType  = MCAN_MEM_TYPE_BUF; /* Use buffer memory type */
    txMessagObject.args       = NULL;

    /* Create TX message object */
    status = CANFD_createMsgObject(gCanfdHandle[CONFIG_MCAN0], &txMessagObject);
    /* Check if message object creation failed */
    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("CANFD: create Tx message object failed\r\n");
        return status;
    }
    
    /* Initialize TX completion semaphore */
    status += SemaphoreP_constructBinary(&gCANFDTxDoneSem, 0);
    if(status)
    {
        DebugP_logError("CANFD: create Tx message semaphore initialization failed\r\n");
        return status;
    }

    /* Store TX message object handle */
    txMessageObjHandle = &txMessagObject;

    return SystemP_SUCCESS;
}

/**
 * @brief Create CAN FD receive message object
 * @param id Message ID type (Standard/Extended)
 * @param startMsgID Starting message ID for reception filtering
 * @param endMsgID Ending message ID for range filtering
 * @param dataLen Expected payload data length
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_Recv_MsgObjectCreate(CANFD_MCANXidType id, uint32_t startMsgID,
                                   uint32_t endMsgID, uint32_t dataLen)
{
    int32_t status = SystemP_SUCCESS;
    gCANFDErrorState = 0;  /* Clear previous errors */
    
    /* Validate data length */
    if (CANFD_ValidateDataLen(dataLen) != SystemP_SUCCESS)
    {
        DebugP_logError("CANFD_Recv_MsgObjectCreate: invalid dataLen=%u\r\n", dataLen);
        return CANFD_INVALID_DATA_LEN_ERR;
    }
    
    /* Initialize and configure RX message object */
    (void)memset(&rxMessagObject, 0, sizeof(rxMessagObject));
    rxMessagObject.direction  = CANFD_Direction_RX;
    rxMessagObject.msgIdType  = id;
    rxMessagObject.startMsgId = startMsgID;      /* Start ID for filtering */
    rxMessagObject.endMsgId   = endMsgID;        /* End ID for range filtering */
    rxMessagObject.rxMemType  = MCAN_MEM_TYPE_BUF; /* Use dedicated RX buffers */
    rxMessagObject.dataLength = dataLen;
    /* Point args to receive buffer where driver will store received data */
    rxMessagObject.args = (uint8_t *)gCANReadBuff;

    /* Create RX message object */
    status = CANFD_createMsgObject(gCanfdHandle[CONFIG_MCAN0], &rxMessagObject);

    /* Check if message object creation failed */
    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("CANFD: create Rx message object failed\r\n");
        return status;
    }

    /* Initialize RX completion semaphore */
    status += SemaphoreP_constructBinary(&gCANFDRxDoneSem, 0);
    if(status)
    {
        DebugP_logError("CANFD: create Rx message semaphore initialization failed\r\n");
        return status;
    }
    
    /* Store RX message object handle */
    rxMessageObjHandle = &rxMessagObject;
    
    return SystemP_SUCCESS;
}

/**
 * @brief Transmit data using configured TX message object
 * @param txdata Pointer to data buffer to transmit
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_TransmitData(const uint8_t *txdata)
{
    int32_t status = SystemP_SUCCESS;
    gCANFDErrorState = 0;  /* Clear previous errors */
    
    /* Validate input parameters */
    if (txdata == NULL)
    {
        DebugP_logError("CANFD_TransmitData: txdata is NULL\r\n");
        return SystemP_FAILURE;
    }

    /* Use the configured message ID from the TX object */
    const uint32_t msgId = txMessageObjHandle->startMsgId;

    /* Transmit data using CAN FD frame type */
    status = CANFD_write(txMessageObjHandle,
                         msgId,
                         CANFD_MCANFrameType_FD, /* Use FD frame format */
                         0U,                      /* Reserved flags */
                         txdata);
                         
    /* Wait for transmission completion with timeout */
    status = SemaphoreP_pend(&gCANFDTxDoneSem, xSemaphoreTXTimeOut);
    
    /* Check if transmission failed */
    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("CANFD_TransmitData: Write operation failed\r\n");
        return status;
    }

    return SystemP_SUCCESS;
}


/* Receive data using the RX message object */
int32_t CANFD_ReceiveData(uint8_t *rxdata)
{
    CANFD_Object*           ptrCanFdObj;
    int32_t status;
    
    gCANFDErrorState = 0;
    
    /* Validate input parameters */
    if (rxMessageObjHandle == NULL) 
    {
        DebugP_logError("CANFD_ReceiveData: RX message object not created\r\n");
        return SystemP_FAILURE;
    }
    if (rxdata == NULL) 
    {
        DebugP_logError("CANFD_ReceiveData: rxdata is NULL\r\n");
        return SystemP_FAILURE;
    }

    ptrCanFdObj = (CANFD_Object*)rxMessageObjHandle->canfdHandle->object;
    
    /* Clear CANFD RX Buffer elements */
    memset(ptrCanFdObj->rxBuffElem.data,0,rxMessageObjHandle->dataLength);
    
    /* 3) Read from the RX message object (index depends on BUF/FIFO configuration) */
    status = CANFD_read(rxMessageObjHandle, 0U, rxdata);

    if (status != SystemP_SUCCESS) 
    {
        DebugP_logError("CANFD_ReadOperation Fail\r\n");
        return status;
    }
    /* 1) Wait for ISR to signal fresh data, with timeout (ms → ticks) */
    status = SemaphoreP_pend(&gCANFDRxDoneSem,xSemaphoreRXTimeOut);
    
    if (status == SystemP_TIMEOUT) 
    {
        DebugP_logError("RX_Semaphore Time-Out Event\r\n");
        return SystemP_TIMEOUT;
    }
    if (status != SystemP_SUCCESS) 
    {
        DebugP_logError("CANFD_ReceiveData: pend failed (%d)\r\n", status);
        return status;
    }

    /* 2) Gate the read on ISR-set flag; avoid stale reads */
    if (gCANRxISRFlage == 0U) 
    {
        /* Spurious wake-up or unrelated event; do not read */
        return SystemP_FAILURE;
    }

    /* Clear the flag BEFORE reading to avoid double-processing if another ISR fires */
    gCANRxISRFlage = 0U;
    
    return SystemP_SUCCESS;
}

/**
 * @brief Delete TX message object and free resources
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_deleteTXObject(void)
{
    if (txMessageObjHandle == NULL)
    {
        DebugP_logError("Delete TX: handle is NULL\r\n");
        return SystemP_FAILURE;
    }
    /* Delete the TX message object */
    int32_t status = CANFD_deleteMsgObject(txMessageObjHandle);
    /* Check if deletion was successful */
    if (status == SystemP_SUCCESS)
    {
        txMessageObjHandle = NULL;  /* Prevent reuse after deletion */
    }
    return status;
}

/**
 * @brief Delete RX message object and free resources
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_deleteRXObject(void)
{
    /* Validate input parameters */
    if (rxMessageObjHandle == NULL)
    {
        DebugP_logError("Delete RX: handle is NULL\r\n");
        return SystemP_FAILURE;
    }
    /* Delete the RX message object */
    int32_t status = CANFD_deleteMsgObject(rxMessageObjHandle);
    /* Check if deletion was successful */
    if (status == SystemP_SUCCESS)
    {
        rxMessageObjHandle = NULL;  /* Prevent reuse after deletion */
    }
    return status;
}

/**
 * @brief Get handle to TX message object
 * @return Pointer to TX message object handle
 */
CANFD_MsgObjHandle CANFD_getTX_Msg_Handle(void)
{
    /* Return the TX message object handle */
    return (CANFD_MsgObjHandle)txMessageObjHandle;
}

/**
 * @brief Get handle to RX message object
 * @return Pointer to RX message object handle
 */
CANFD_MsgObjHandle CANFD_getRX_Msg_Handle(void)
{
    /* Return the RX message object handle */
    return (CANFD_MsgObjHandle)rxMessageObjHandle;
}

/**
 * @brief CAN FD error callback function
 * @param args Callback arguments (unused)
 * @param reason Error reason code from CAN FD peripheral
 */
void App_CANFDErrorCallback(void *args, CANFD_Reason reason)
{
    (void)args;  /* Suppress unused parameter warning */
    gCANFDErrorState = 0;  /* Clear previous error state */
    
    /* Handle specific error conditions and update error state bitmap */
    if(reason == CANFD_Reason_TX_CANCELED)
    {
        /* Set the error state bit for TX cancellation */
        gCANFDErrorState |= (1 << CANFD_Reason_TX_CANCELED);
        DebugP_logError("CANFD_TX_CANCELED\n");
    }
    
    if(reason == CANFD_Reason_ECC_ERROR)
    {
        /* Set the error state bit for ECC error */
        gCANFDErrorState |= (1 << CANFD_Reason_ECC_ERROR);
        DebugP_logError("CANFD_ECC_ERROR\n");
    }

    if(reason == CANFD_Reason_BUSOFF)
    {
        /* Set the error state bit for bus off */
        gCANFDErrorState |= (1 << CANFD_Reason_BUSOFF);
        DebugP_logError("CANFD_BUSOFF\n");
    }

    if(reason == CANFD_Reason_PROTOCOL_ERR_DATA_PHASE)
    {
        /* Set the error state bit for protocol error in data phase */
        gCANFDErrorState |= (1 << CANFD_Reason_PROTOCOL_ERR_DATA_PHASE);
        DebugP_logError("CANFD_ERR_DATA_PHASE\n");
    }

    if(reason == CANFD_Reason_PROTOCOL_ERR_ARB_PHASE)
    {
        /* Set the error state bit for protocol error in arbitration phase */
        gCANFDErrorState |= (1 << CANFD_Reason_PROTOCOL_ERR_ARB_PHASE);
        DebugP_logError("CANFD_ARBITRATION_PHASE_ERROR\n");
    }
    
    /* Limit error state to 8 bits for consistency */
    gCANFDErrorState &= 0xFFU;
}

/**
 * @brief Get current CAN FD configuration parameters
 * @param defaultParams Output buffer to store configuration parameters (52 bytes)
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_GetDefaultParams(uint8_t *defaultParams)
{
    /* Validate input parameters */
    if (defaultParams == NULL) 
    {
        return SystemP_FAILURE;
    }
    
    const CANFD_OpenParams *op = &openParams[CONFIG_MCAN0];
    const CANFD_Config     *cfg = &gCanfdConfig[CONFIG_MCAN0];

    if ((op == NULL) || (cfg == NULL) || (cfg->attrs == NULL)) 
    {
        return SystemP_FAILURE;
    }
    
    /* Pack CAN FD configuration parameters into byte array (big-endian) */
    convert32To8_BE_ptr(op->fdMode, &defaultParams[0]);
    convert32To8_BE_ptr(op->brsEnable, &defaultParams[4]);
    convert32To8_BE_ptr(op->darEnable, &defaultParams[8]);
    
    /* Pack bit timing parameters into byte array (big-endian) */
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.nomBrp, &defaultParams[12]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.nomPropSeg, &defaultParams[16]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.nomPseg1, &defaultParams[20]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.nomPseg2, &defaultParams[24]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.nomSjw, &defaultParams[28]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.dataBrp, &defaultParams[32]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.dataPropSeg, &defaultParams[36]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.dataPseg1, &defaultParams[40]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.dataPseg2, &defaultParams[44]);
    convert32To8_BE_ptr(cfg->attrs->CANFDMcanBitTimingParams.dataSjw, &defaultParams[48]);
    return SystemP_SUCCESS;
}

/**
 * @brief Get current CAN FD failure reason
 * @return Error state bitmap (8-bit)
 */
int32_t CANFD_GetFailureReason(void)
{
    return (gCANFDErrorState);
}

/**
 * Pack a byte buffer into a uint32_t buffer (big-endian per 32-bit word).
 * Each group of 4 bytes becomes one uint32_t:
 *   word = b0<<24 | b1<<16 | b2<<8 | b3
 * If len is not a multiple of 4, the last word is zero-padded.
 *
 * @param inBuff   Pointer to input bytes.
 * @param outBuff  Pointer to output uint32_t array.
 * @param len      Number of bytes in input.
 * @param outCapacityWords   Expected outbut 32bit words.
 * @return         Number of 32-bit words written (>=0), or SYSTEMP_FAILURE (-1) on error.
 */
int32_t u8_to_u32_convert(const uint8_t *inBuff, uint32_t *outBuff, size_t len, size_t outCapacityWords)
{
    if (inBuff == NULL || outBuff == NULL) 
    {
        return SystemP_FAILURE;
    }

    size_t byteID = 0;
    int32_t wordCount = 0;

    while (byteID < len) 
    {
        uint32_t b0 = (byteID + 0 < len) ? (uint32_t)inBuff[byteID + 0] : 0u;
        uint32_t b1 = (byteID + 1 < len) ? (uint32_t)inBuff[byteID + 1] : 0u;
        uint32_t b2 = (byteID + 2 < len) ? (uint32_t)inBuff[byteID + 2] : 0u;
        uint32_t b3 = (byteID + 3 < len) ? (uint32_t)inBuff[byteID + 3] : 0u;

        outBuff[wordCount] = (b0 << 24) | (b1 << 16) | (b2 << 8) | (b3);
        byteID += 4;
        wordCount++;
    }

    // Out of range wordcount
    if(wordCount > outCapacityWords) return SystemP_FAILURE;
    return wordCount;
}

/**
 * @brief Convert 32-bit value to 8-bit big-endian array
 * @param data 32-bit value to convert
 * @param buffer Output buffer (must have space for 4 bytes)
 */
void convert32To8_BE_ptr(uint32_t data, uint8_t *buffer) 
{
    *buffer++ = (uint8_t)((data >> 24) & 0xFF); /* Most significant byte */
    *buffer++ = (uint8_t)((data >> 16) & 0xFF);
    *buffer++ = (uint8_t)((data >> 8) & 0xFF);
    *buffer   = (uint8_t)(data & 0xFF);         /* Least significant byte */
}

/**
 * @brief Close CAN FD peripheral and cleanup resources
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t CANFD_Close(void)
{
    if(gCanfdHandle[CONFIG_MCAN0] == NULL) return SystemP_FAILURE;
    
    /* Delete message objects */
    CANFD_deleteTXObject();
    CANFD_deleteRXObject();
    
    /* Close CAN FD handle */
    CANFD_close(gCanfdHandle[CONFIG_MCAN0]);
    
    /* Cleanup synchronization objects */
    SemaphoreP_destruct(&gCANFDTxDoneSem);
    
    /* Clear handle to prevent reuse */
    gCanfdHandle[CONFIG_MCAN0] = NULL;
    return SystemP_SUCCESS;
}

/**
 * @brief CAN FD transfer completion callback function
 * @param args Callback arguments (unused)
 * @param reason Transfer reason code from CAN FD peripheral
 */
void App_CANFDTransferCallback(void *args, CANFD_Reason reason)
{
    if (reason == CANFD_Reason_TX_COMPLETION)
    {
        /* Signal TX completion and set ISR flag */
        SemaphoreP_post((SemaphoreP_Object *)&gCANFDTxDoneSem);
        /* Set the TX ISR flag */
        gCANTxISRFlage = 1;
    }
    if (reason == CANFD_Reason_RX)
    {
        /* Set RX ISR flag (RX semaphore handling disabled) */
        SemaphoreP_post((SemaphoreP_Object *)&gCANFDRxDoneSem);
       gCANRxISRFlage = 1;
    }
}
