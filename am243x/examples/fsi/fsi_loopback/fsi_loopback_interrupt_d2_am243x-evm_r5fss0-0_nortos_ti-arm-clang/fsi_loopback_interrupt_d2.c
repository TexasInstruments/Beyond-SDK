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

/*
 * This example performs FSI (Node) TX to FSI RX internal loopback in interrupt mode.
 * The application configures an instance of FSI TX and FSI RX module with below configuration
 *
 * - Single lane
 * - TX clock at 50 MHz
 * - 16 words per frame (transfer)
 * - Register both FSI TX interrupt 1 and FSI RX interrupt 1
 *
 * With above configuration, the application does the following:
 * FSI handshake, then, the application transfers 100 frames of data from FSI TX,
 * waits for data to be received by FSI RX and then compares the data.
 * Once the transfer it completes, it compares the source and destination buffers for any data mismatch.
 */

/* FSI TXCLK - 50 MHz */
#define FSI_APP_TXCLK_FREQ              (1 * 1000 * 1000)
/* FSI module input clock - 500 MHz */
#define FSI_APP_CLK_FREQ                (CONFIG_FSI_TX0_CLK)
/* FSI TX prescaler value for TXCLKIN of 100 MHz. / 2 is provided as TXCLK = TXCLKIN/2 */
#define FSI_APP_TX_PRESCALER_VAL        (FSI_APP_CLK_FREQ / FSI_APP_TXCLK_FREQ / 2U)

#define FSI_APP_LOOP_COUNT              (100U)
/* User data to be sent with Data frame */
#define FSI_APP_TX_USER_DATA            (0x07U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE    (16U)
/* 0x0U for 1 lane and 0x1U for two lanes */
#define FSI_APP_N_LANES                 (0x0U)
#define FSI_APP_TX_DATA_FRAME_TAG       (0x1U)

/* Index of FSI RX buffer, gBufIdx + FSI_APP_FRAME_DATA_WORD_SIZE should be <= 16 */
uint16_t gRxBufData[FSI_MAX_VALUE_BUF_PTR_OFF + 1U];

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

    /* Test parameters */
    rxBaseAddr = CONFIG_FSI_RX0_BASE_ADDR;
    txBaseAddr = CONFIG_FSI_TX0_BASE_ADDR;
    dataSize   = FSI_APP_FRAME_DATA_WORD_SIZE;
    loopCnt    = FSI_APP_LOOP_COUNT;
    bufIdx     = 0U;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    
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

    /* TODO: Check if still need it */
    FSI_setRxBufferPtr(rxBaseAddr, 0U);
    FSI_getRxEventStatus(rxBaseAddr, &rxEventSts);
    FSI_clearRxEvents(rxBaseAddr, rxEventSts);

    /* Start transfer */
    while(loopCnt--)
    {
        /* Memset TX buffer with new data for every loop */
        for(uint32_t i = 0; i < dataSize; i++)
        {
            gRxBufData[i] = 0U;
        }
        /* Wait RX completion */
        SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);

        /* Read data */
        status = FSI_readRxBuffer(rxBaseAddr, gRxBufData, dataSize, bufIdx);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Transmit data */
        status += FSI_writeTxBuffer(txBaseAddr, gRxBufData, dataSize, bufIdx);
        status += FSI_startTxTransmit(txBaseAddr);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Wait for TX completion */
        SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
        DebugP_log("Node RX/TX finished, %d loopcnt\r\n", loopCnt);

        status += FSI_setTxBufferPtr(txBaseAddr, bufIdx);
        status += FSI_setRxBufferPtr(rxBaseAddr, bufIdx);

    }

    Fsi_appIntrDeInit(txBaseAddr, rxBaseAddr);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[FSI] %d frames successfully received!!!\r\n", FSI_APP_LOOP_COUNT);
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

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