/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/fsi.h>
#include <drivers/pinmux.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <board/ioexp/ioexp_tca6424.h>
#include "sbl_fsi_d2.h"

/*
 * This example performs SBL FSI (Node) TX to FSI RX in interrupt mode.
 * The application configures an instance of FSI TX and FSI RX module with below configuration
 *
 * - Single lane
 * - TX clock at 50 MHz
 * - 16 words per frame (transfer)
 * - Register both FSI TX interrupt 1 and FSI RX interrupt 1
 *
 * With above configuration, the application does the following:
 * FSI handshake, then does application handshake with a MAGIC WORD, receives the application image and sends it back to Lead device.
 * After completing the transfer, it sends a MAGIC WORD to indicate that the application image has been received.
 */

/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (40 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_TX0_CLK)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lanes */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)
/* MAGIC WORD that indicates application transfer starts */
#define APP_MAGIC_WORD_START   (0xC0DEFEED)
#define APP_MAGIC_WORD_END   (0xCAFEBABE)

/* Index of FSI RX buffer, gBufIdx + FSI_APP_FRAME_DATA_WORD_SIZE should be <= 16 */
uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x20000U)
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE]__attribute__((aligned(128), section(".bss.filebuf")));

/* Globals */
uint16_t txEventSts = 0, rxEventSts = 0;

volatile uint32_t fsiTxInt1Received = 0;
volatile uint32_t fsiRxInt1Received = 0;
uint32_t rxTimeOutCntr = 0x100000;
uint32_t error = 0;

static HwiP_Object gFsiTxHwiObject, gFsiRxHwiObject;
static SemaphoreP_Object gFsiTxSemObject, gFsiRxSemObject;

/* Function Prototypes */
static inline void compare16(uint16_t val1, uint16_t val2);
/* This function performs a handshake between two nodes in a FSI (Flexible Serial Interface) network */
void handshake_node(uint32_t txBaseAddr, uint32_t rxBaseAddr);
/* This function checks the received frame type and tag for validation */
void checkReceivedFrameTypeTag(FSI_FrameType type, FSI_FrameTag tag, uint32_t rxBaseAddr);
/* This function checks received FSI dataframes for APP_MAGIC_WORD_START and extracts file size */
int32_t checkForMagicWordAndFileSize(const uint16_t *buf, uint32_t *fileSize);
/* This function initializes basic FSI settings */
void initFSI(uint32_t txBaseAddr,uint32_t rxBaseAddr);
/* This function configures FSI TX parameters */
static int32_t Fsi_appTxConfig(uint32_t txBaseAddr);
/* This function configures FSI RX parameters */
static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr);

/* This function initializes and deinitializes FSI interrupts */
static int32_t Fsi_appIntrInit(uint32_t txBaseAddr, uint32_t rxBaseAddr);
static void Fsi_appIntrDeInit(uint32_t txBaseAddr, uint32_t rxBaseAddr);
/* This function handles FSI TX interrupt callback */
static void Fsi_appTxCallback(void *args);
/* This function handles FSI RX interrupt callback */
static void Fsi_appRxCallback(void *args);

static TCA6424_Config  gTCA6424_Config;

/* This function configures the I2C IO expander for FSI connection */
void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    uint32_t            ioIndex;

    if(status == SystemP_SUCCESS)
    {
        /* IO expander I2C 0x22 - Pin8 (P07) --> FSI_FET_SEL. LOW -> HSE connector, HIGH -> EVM J7 header (C2000 connection) */
        ioIndex = 7;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
    }
    TCA6424_close(&gTCA6424_Config);
}

/* This function is the main FSI node application entry point */
void *fsi_node_main(void *args)
{
    int32_t     status =0;
    uint32_t    rxBaseAddr, txBaseAddr;
    uint16_t    dataSize;
    uint32_t    loopCnt;
    uint16_t    bufIdx;
    uint16_t    wordSizeBytes = 2;
    uint16_t    dataframeBytes = 0;
    uint32_t    fileSizeBytes = 0;
    uint32_t    remainderBytes = 0;
    uint32_t    printCounter = 0;

    /* Test parameters */
    rxBaseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    txBaseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    dataSize   = FSI_APP_FRAME_DATA_WORD_SIZE;
    bufIdx     = 0U;

    dataframeBytes = dataSize * wordSizeBytes;
    
    /* Configure the IO Expander to connect FSI to J7 EVM header */
    i2c_io_expander(NULL);

    DebugP_log("FSI Node - Loopback Interrupt application started at %dHz...\r\n",FSI_APP_TXCLK_FREQ);
 
    /* Initialize basic settings for FSI */
    initFSI(txBaseAddr,rxBaseAddr);
    status += Fsi_appIntrInit(txBaseAddr, rxBaseAddr);

    /* Handshake with Lead device */
    handshake_node(txBaseAddr, rxBaseAddr);
    DebugP_log("FSI Node - Handshake done\r\n");
    /* Disable RX Ping Interrupt and RX events cleanup */
    FSI_disableRxInterrupt(rxBaseAddr, FSI_INT1, FSI_RX_EVT_PING_FRAME);
    ClockP_usleep(100);

    /* FSI configuration */
    status  = Fsi_appTxConfig(txBaseAddr);
    status += Fsi_appRxConfig(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Send Flush Sequence to sync after every RX soft reset */
    status = FSI_executeTxFlushSequence(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    DebugP_assert(status == SystemP_SUCCESS);

    /* PC-- trying to fix first RX which is wrong */
    FSI_setRxBufferPtr(rxBaseAddr, 0U);
    FSI_getRxEventStatus(rxBaseAddr, &rxEventSts);
    FSI_clearRxEvents(rxBaseAddr, rxEventSts);

    /* Start listening for MAGIC WORD and file size */
    for(uint32_t j = 0; j < dataSize; j++)
    {
        gRxBufData[j] = 0U;
    }
    /* Wait RX completion */
    SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);
    /* Read data */
    status = FSI_readRxBuffer(rxBaseAddr, gRxBufData, dataSize, bufIdx);
    DebugP_assert(status == SystemP_SUCCESS);
    status = checkForMagicWordAndFileSize(gRxBufData, &fileSizeBytes);
    loopCnt = fileSizeBytes / dataframeBytes;
    remainderBytes = fileSizeBytes % dataframeBytes;

    /* Offset for copying into gAppImageBuf */
    uint32_t appImageOffset = 0;
    DebugP_log("FSI Node - fileSizeBytes: %d, dataframeBytes: %d\r\n", fileSizeBytes, dataframeBytes);
    DebugP_log("FSI Node - loopCnt: %d, remainderBytes: %d\r\n", loopCnt, remainderBytes);

    /* Start receiving application data for all frames including remainder */
    uint32_t totalFrames = loopCnt + ((remainderBytes > 0) ? 1 : 0);
    for(uint32_t i = 0; i < totalFrames; i++)
    {
        uint16_t currWords;
        uint16_t currBytes;

        if(i < loopCnt)
        {
            currWords = dataSize;
            currBytes = dataframeBytes;
        }
        else
        {
            /* Last frame: handle remainder */
            currWords = remainderBytes / wordSizeBytes;
            if(remainderBytes % wordSizeBytes != 0)
                currWords++; /* round up */
            currBytes = remainderBytes;
            if(currBytes == 0)
                break; /* no remainder to process */
        }
        /* Wait RX completion */
        SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);

        /* Read data */
        status = FSI_readRxBuffer(rxBaseAddr, gRxBufData, currWords, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Copy received data to gAppImageBuf */
        memcpy(&gAppImageBuf[appImageOffset], gRxBufData, currWords * wordSizeBytes);
        appImageOffset += currWords * wordSizeBytes;

        /* Transmit data */
        status += FSI_writeTxBuffer(txBaseAddr, gRxBufData, currWords, bufIdx);
        status += FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Wait for TX completion */
        SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);

        if(i < loopCnt)
            DebugP_log("FSI Node - RX/TX dataframe loopCnt:%d\r\n", i);
        else
            DebugP_log("FSI Node - RX/TX finished receiving remainder words: %d\r\n", currWords);

        status += FSI_setTxBufferPtr(txBaseAddr, bufIdx);
        status += FSI_setRxBufferPtr(rxBaseAddr, bufIdx);
    }

    /* Send back MAGIC WORD to signal application was received */
    {
        uint16_t magicBuf[2];
        magicBuf[0] = (uint16_t)((APP_MAGIC_WORD_END >> 16) & 0xFFFF);
        magicBuf[1] = (uint16_t)(APP_MAGIC_WORD_END & 0xFFFF);
        
        status = FSI_writeTxBuffer(txBaseAddr, magicBuf, 2, bufIdx);
        status += FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Wait for TX completion */
        SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
        DebugP_log("FSI Node - Sent MAGIC WORD - Application received\r\n");
    }

    Fsi_appIntrDeInit(txBaseAddr, rxBaseAddr);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("FSI Node - frames successfully received!!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    return NULL;
}

/* This function initializes basic FSI settings */
void initFSI(uint32_t txBaseAddr,uint32_t rxBaseAddr)
{
    DebugP_log("FSI Node - initFSI\r\n");
     int32_t status;
    /* Disable loopback */
    status = FSI_disableRxInternalLoopback(rxBaseAddr);
    status = FSI_performTxInitialization(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);
    status = FSI_performRxInitialization(rxBaseAddr);
    DebugP_assert(status == SystemP_SUCCESS);
}
/* This function configures FSI TX parameters */
static int32_t Fsi_appTxConfig(uint32_t txBaseAddr)
{
    int32_t     status;

    /* Setting for requested transfer params */
    status = FSI_setTxSoftwareFrameSize(txBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setTxDataWidth(txBaseAddr, FSI_APP_N_LANES);

    /* Setting frame config */
    status += FSI_setTxUserDefinedData(txBaseAddr, FSI_APP_TX_USER_DATA);
    status += FSI_setTxFrameTag(txBaseAddr, FSI_APP_TX_DATA_FRAME_TAG);
    status += FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_NWORD_DATA);
    status += FSI_setRxBufferPtr(txBaseAddr, 0U);

    return status;
}

/* This function configures FSI RX parameters */
static int32_t Fsi_appRxConfig(uint32_t rxBaseAddr)
{
    int32_t     status;

    /* Setting for requested transfer params */
    status = FSI_setRxSoftwareFrameSize(rxBaseAddr, FSI_APP_FRAME_DATA_WORD_SIZE);
    status += FSI_setRxDataWidth(rxBaseAddr, FSI_APP_N_LANES);
    status += FSI_setRxBufferPtr(rxBaseAddr, 0U);

    return status;
}

/* This function initializes FSI interrupts */
static int32_t Fsi_appIntrInit(uint32_t txBaseAddr, uint32_t rxBaseAddr)
{
    int32_t     status;
    uint32_t    txIntrNum, rxIntrNum;
    HwiP_Params txHwiPrms, rxHwiPrms;

    /*
     * TX interrupt config and registration
     */
    txIntrNum = CONFIG_FSI_TX0_INTR1;
    status = SemaphoreP_constructBinary(&gFsiTxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&txHwiPrms);
    txHwiPrms.intNum = txIntrNum;
    txHwiPrms.callback = Fsi_appTxCallback;
    txHwiPrms.args = (void *) txBaseAddr;
    HwiP_construct(&gFsiTxHwiObject, &txHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable TX frame done interrupt */
    status += FSI_enableTxInterrupt(txBaseAddr, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
    
    /*
     * RX interrupt config and registration
     */
    rxIntrNum = CONFIG_FSI_RX0_INTR1;
    status = SemaphoreP_constructBinary(&gFsiRxSemObject, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    HwiP_Params_init(&rxHwiPrms);
    rxHwiPrms.intNum = rxIntrNum;
    rxHwiPrms.callback = Fsi_appRxCallback;
    rxHwiPrms.args = (void *) rxBaseAddr;
    HwiP_construct(&gFsiRxHwiObject, &rxHwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Enable RX frame done interrupt */
    status += FSI_enableRxInterrupt(rxBaseAddr, FSI_INT1, FSI_RX_EVT_DATA_FRAME | FSI_RX_EVT_PING_FRAME);
 
    return status;
}

/* This function deinitializes FSI interrupts */
static void Fsi_appIntrDeInit(uint32_t txBaseAddr, uint32_t rxBaseAddr)
{
    /* TX interrupt deinit */
    FSI_disableTxInterrupt(txBaseAddr, FSI_INT1, FSI_TX_EVTMASK);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK);
    HwiP_destruct(&gFsiTxHwiObject);
    SemaphoreP_destruct(&gFsiTxSemObject);

    /* RX interrupt deinit */
    FSI_disableRxInterrupt(rxBaseAddr, FSI_INT1, FSI_TX_EVTMASK);
    FSI_clearRxEvents(rxBaseAddr, FSI_RX_EVTMASK);
    HwiP_destruct(&gFsiRxHwiObject);
    SemaphoreP_destruct(&gFsiRxSemObject);

    return;
}

/* This function handles FSI TX interrupt callback */
static void Fsi_appTxCallback(void *args)
{
    uint32_t txBaseAddr = (uint32_t) args;

    fsiTxInt1Received = 1U;
    FSI_getTxEventStatus(txBaseAddr, &txEventSts);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK);
    SemaphoreP_post(&gFsiTxSemObject);

    return;
}

/* This function handles FSI RX interrupt callback */
static void Fsi_appRxCallback(void *args)
{
    uint32_t rxBaseAddr = (uint32_t) args;

    fsiRxInt1Received = 1U;
    FSI_getRxEventStatus(rxBaseAddr, &rxEventSts);
    FSI_clearRxEvents(rxBaseAddr, rxEventSts);
    SemaphoreP_post(&gFsiRxSemObject);

    return;
}

/* This function compares two 16-bit values and increments error counter if they don't match */
static inline void compare16(uint16_t val1, uint16_t val2)
{
    if(val1 != val2)
    {
        error++;
    }
}

/* This function checks the received frame type and tag for validation */
void checkReceivedFrameTypeTag(FSI_FrameType type, FSI_FrameTag tag, uint32_t rxBaseAddr)
{
    uint16_t pingTag, frameTag;
    FSI_FrameType frameType;
    FSI_getRxFrameType(rxBaseAddr, &frameType);
    FSI_getRxPingTag(rxBaseAddr, &pingTag);
    FSI_getRxFrameTag(rxBaseAddr, &frameTag);

    compare16((uint16_t)frameType, (uint16_t)type);

    if(type == FSI_FRAME_TYPE_PING)
    {
        compare16((uint16_t)pingTag, (uint16_t)tag);
    }
    else
    {
        compare16((uint16_t)frameTag , (uint16_t)tag);
    }
}

/* This function performs a handshake between two nodes in a FSI (Flexible Serial Interface) network */
void handshake_node(uint32_t txBaseAddr, uint32_t rxBaseAddr)
{

    DebugP_log("FSI Node - Starting handshake\r\n");
    while(1)
    {
        /* Wait RX completion */
        SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);
        compare16(rxEventSts, (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));
        checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, FSI_FRAME_TAG0,rxBaseAddr);
        /*
         * If received frame type and tag matches, exit this loop and proceed to
         * next step by sending flush sequence, otherwise clear error and
         * interrupt flag and continue looping.
         */
        if(error == 0)
        {
            fsiRxInt1Received = 0;
            break;
        }

        fsiRxInt1Received = 0;
        error = 0;
    }

    while(1)
    {
        /*
         * Send the flush sequence
         */
        FSI_executeTxFlushSequence(txBaseAddr, FSI_APP_TX_PRESCALER_VAL);

        /*
         * Send a ping frame with frame tag 0000b
         */
        FSI_setTxFrameTag(txBaseAddr, FSI_FRAME_TAG0);
        FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_PING);
        FSI_startTxTransmit(txBaseAddr);

        /* Wait RX completion */
        if (SemaphoreP_pend(&gFsiRxSemObject, 10U) != SystemP_SUCCESS){
            /* resend packet in case of timeout or failure */
            if (SemaphoreP_pend(&gFsiRxSemObject, 10U) == SystemP_TIMEOUT){
                DebugP_log("[FSI] Node - TAG0 timeout\r\n");
            }
            continue;
        } else {
        compare16(rxEventSts, (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));
        checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, FSI_FRAME_TAG1,rxBaseAddr);
        /*
         * If received frame type and tag matches, exit this loop and proceed
         * to next step by sending flush sequence, otherwise clear error and
         * interrupt flag and continue looping.
         */
        if(error == 0)
        {
            fsiRxInt1Received = 0;
            break;
        }

        fsiRxInt1Received = 0;
        error = 0;
        }
    }
    /*
     * Send a ping frame with frame tag 0001b
     */
    FSI_setTxFrameTag(txBaseAddr, FSI_FRAME_TAG1);
    FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(txBaseAddr);
    ClockP_usleep(10);
}

/* This function checks received FSI dataframes for APP_MAGIC_WORD_START and extracts file size */
int32_t checkForMagicWordAndFileSize(const uint16_t *buf, uint32_t *fileSize)
{
        /* Combine two 16-bit words to form 32-bit magic word */
        uint32_t word = ((uint32_t)buf[1] << 16) | buf[0];
        DebugP_log("[FSI] Node - checking magic word buf[0]=0x%x , buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\r\n", buf[0], buf[1], buf[2], buf[3]);
        if(word == APP_MAGIC_WORD_START)
        {
            /* Next 16 bits (2 bytes) is file size (little endian) */
            *fileSize = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[2]);
            DebugP_log("[FSI] Node - Received MAGIC WORD 0x%08X, file size: %u bytes\r\n", word, *fileSize);
            return SystemP_SUCCESS;
        } else {
                DebugP_log("[FSI] Node - MAGIC WORD mismatch. Received 0x%08X, file size: %u bytes\r\n", word, *fileSize);
        }
    return SystemP_FAILURE;
}


