/*
 *  Copyright (C) 2024-25 Texas Instruments Incorporated
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

/*!
 * \file  fota_enet.c
 *
 * \brief This file contains the ethernet FOTA implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "fota_enet.h"
#include <networking/enet/core/utils/include/enet_ethpatterns.h>
#include <networking/enet/core/include/core/enet_utils.h>
#include "ti_enet_config.h"
#include <board/flash.h>
#include "ti_enet_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_LPBK_ETHERTYPE_IPV4 (0x0800)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool EnetFOTA_parseFrame(EthFrame *frame,
                               uint32_t dataLen);

static uint32_t EnetFOTA_retrieveFreeTxPkts(void);

static uint32_t EnetFOTA_receivePkts(void);

static int32_t EnetFOTA_setupCpswAle(void);

static void EnetFOTA_closeEnet(void);

static int32_t EnetFOTA_showAlivePhys(void);

static int32_t EnetFOTA_waitForLinkUp(void);

static int32_t EnetFOTA_macMode2PhyMii(emac_mode macMode,
                                       EnetPhy_Mii *mii);

static void EnetFOTA_macMode2MacMii(emac_mode macMode,
                                    EnetMacPort_Interface *mii);

static void EnetFOTA_rxIsrFxn(void *appData);

static void EnetFOTA_txIsrFxn(void *appData);

static void EnetFOTA_initPkt(EnetDma_Pkt *pPktInfo,
                               uint8_t *udpPayload);

static void EnetFOTA_initTxFreePktQ(void);

static void EnetFOTA_initRxReadyPktQ(void);

static int32_t EnetFOTA_openDma(void);

static void EnetFOTA_closeDma(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

const EnetFOTA_AddrInfo enetAppAddrInfo =
{
    .dstMac     = ENET_HOST_PC_MAC_ADDRESS,
    .srcIP      = ENET_SOURCE_IP_ADDRESS,
    .dstIP      = ENET_DESTINATION_IP_ADDRESS,
    .srcPortUDP = ENET_PORT,
    .dstPortUDP = ENET_PORT,
};

const uint32_t EnetFOTA_MagicNum  = 0x05B1C00D;
const uint32_t EnetFOTA_Ack       = 0x05B10ACD;

uint8_t gFlashFileBuf[MAX_FILE_SIZE] __attribute__((aligned(128))) __attribute__((section(".ddr_memory")));
uint32_t gFlashFileSize;

EnetFOTA_LLDObj gEnetFOTA_LLDObj;
EnetFOTA_MetaObj gEnetFOTA_MetaObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetFOTA_setup(void)
{
    EnetApp_HandleInfo handleInfo;
    int32_t status = 0;

    gEnetFOTA_LLDObj.macMode          = RGMII;
    gEnetFOTA_LLDObj.boardId          = ENETBOARD_CPB_ID;
    gEnetFOTA_LLDObj.testLoopBackType = ENET_LOOPBACK_TYPE_NONE;


    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gEnetFOTA_LLDObj.enetType, &gEnetFOTA_LLDObj.instId);
    EnetApp_getEnetInstMacInfo(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId, &gEnetFOTA_LLDObj.macPort[0], &gEnetFOTA_LLDObj.numMacPorts);

    /* Create Global Event Object */
    status = EventP_construct(&gEnetFOTA_LLDObj.appEvents);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable peripheral clocks */
    EnetAppUtils_enableClocks(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId);

    /* Create TX/RX semaphores */
    status = SemaphoreP_constructBinary(&gEnetFOTA_LLDObj.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gEnetFOTA_LLDObj.txSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Local core id */
    gEnetFOTA_LLDObj.coreId = EnetSoc_getCoreId();
    EnetApp_driverInit();
    if (status == ENET_SOK)
    {
        status = EnetApp_driverOpen(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId);

        if (status != ENET_SOK)
        {
            EnetAppUtils_print("[ ENETFOTA ] Failed to open Enet driver: %d\r\n", status);
        }
    }


    EnetApp_acquireHandleInfo(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId, &handleInfo);
    gEnetFOTA_LLDObj.hEnet = handleInfo.hEnet;
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId, gEnetFOTA_LLDObj.coreId, &attachCoreOutArgs);
        gEnetFOTA_LLDObj.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA driver */
    if (status == ENET_SOK)
    {
        status = EnetFOTA_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("[ ENETFOTA ] Failed to open DMA: %d\r\n", status);
        }
    }

    EnetAppUtils_print("[ ENETFOTA ] Please wait for Linkup ...\r\n");

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetFOTA_waitForLinkUp();
    }

    if(status != ENET_ETIMEOUT)
        EnetAppUtils_print("[ ENETFOTA ] Linkup Done!\r\n");

    return status;
}

void EnetFOTA_destruct(void)
{
    /* Close Enet DMA driver */
    EnetFOTA_closeDma();

    /* Detach core from all peripherals*/
    EnetApp_coreDetach(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId, gEnetFOTA_LLDObj.coreId,gEnetFOTA_LLDObj.coreKey);

    /* Close Enet Drivers */
    EnetApp_releaseHandleInfo(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId);
    gEnetFOTA_LLDObj.hEnet = NULL;

    /* Deinit Enet driver */
    Enet_deinit();

    /* Disable peripheral clocks */
    EnetAppUtils_disableClocks(gEnetFOTA_LLDObj.enetType, gEnetFOTA_LLDObj.instId);

    /* Delete all TX/RX semaphores */
    SemaphoreP_destruct(&gEnetFOTA_LLDObj.rxSemObj);
    SemaphoreP_destruct(&gEnetFOTA_LLDObj.txSemObj);

    /* Delete Global Event Object */
    EventP_destruct(&gEnetFOTA_LLDObj.appEvents);
}


int32_t EnetFOTA_transferAppimage(void)
{
    EnetDma_Pkt *pktInfoRx;
    EnetDma_Pkt *pktInfoTx;
    EthFrame *frame;
    uint32_t rxReadyCnt;
    uint32_t txFreeCnt;
    bool finished = false;
    bool appPkt = false;
    uint32_t currPktCnt = 0U;
    uint32_t totalPktCnt = 0xFFFFFFFFU;
    uint32_t waitCount = 0;
    int32_t status = ENET_SOK;
    uint32_t EthPayloadLen = 0;
    Bootloader_UniflashFileHeader *fileHeader;

    /* Set appimage packet number to 0 to start transfer */
    gEnetFOTA_MetaObj.appPktNum = 0U;

    while(!finished)
    {
        waitCount++;
        if(waitCount > 10000000 && (totalPktCnt == 0xFFFFFFFFU))
        {
            break;
        }
        SemaphoreP_pend(&gEnetFOTA_LLDObj.rxSemObj, SystemP_NO_WAIT);
        /* Get the packets received so far */
        rxReadyCnt = EnetFOTA_receivePkts();
        if (rxReadyCnt > 0U)
        {
            /* Consume the received packets and release them */
            pktInfoRx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.rxReadyQ);
            while (NULL != pktInfoRx)
            {
                EnetDma_checkPktState(&pktInfoRx->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_READYQ,
                                      ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Consume the packet by just printing its content */
                EthPayloadLen = pktInfoRx->sgList.list[0].segmentFilledLen - ETH_HDR_SIZE - IPV4_HDR_SIZE - UDP_HDR_SIZE;
                frame = (EthFrame *)pktInfoRx->sgList.list[0].bufPtr;

                appPkt = EnetFOTA_parseFrame(frame, EthPayloadLen);
                if(appPkt)
                {
                    /* Check if seq. numbers match with expected and received */
                    if (gEnetFOTA_MetaObj.appPktNum == currPktCnt)
                    {
                        if (currPktCnt == 0U)
                        {
                            /* Initialize total pkt count from pkt 0 */
                            /* It is stored in the rsv1 (reserved) field of the Uniflash header */
                            gFlashFileSize = 0U;
                            fileHeader = (Bootloader_UniflashFileHeader*) &gEnetFOTA_MetaObj.appPktData[MGC_NUM_SIZE+SEQ_NUM_SIZE];
                            totalPktCnt = fileHeader->rsv1;
                            EnetAppUtils_print("[ ENETFOTA ] Receiving file, please wait ...\r\n");
                        }


                        /* WRITING THE BYTES INTO MSRAM */
                        memcpy(&gFlashFileBuf[gFlashFileSize], &gEnetFOTA_MetaObj.appPktData[MGC_NUM_SIZE+SEQ_NUM_SIZE], (EthPayloadLen-MGC_NUM_SIZE-SEQ_NUM_SIZE));
                        gFlashFileSize += (EthPayloadLen-MGC_NUM_SIZE-SEQ_NUM_SIZE);
                        /* Fill the TX payload with seq number and ACK code */
                        memcpy(&gEnetFOTA_MetaObj.txPayload[0], &currPktCnt, sizeof(currPktCnt));
                        memcpy(&gEnetFOTA_MetaObj.txPayload[sizeof(currPktCnt)], &EnetFOTA_Ack, sizeof(EnetFOTA_Ack));

                        /* Get free tx packet from queue */
                        pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
                        while (pktInfoTx == NULL)
                        {
                            txFreeCnt = EnetFOTA_retrieveFreeTxPkts();
                            if (txFreeCnt == 0)
                            {
                                SemaphoreP_pend(&gEnetFOTA_LLDObj.txSemObj, SystemP_WAIT_FOREVER);
                            }
                            pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
                        }

                        /* Init TX packet header */
                        EnetFOTA_initPkt(pktInfoTx, gEnetFOTA_MetaObj.txPayload);

                        EnetDma_checkPktState(&pktInfoTx->pktState,
                                              ENET_PKTSTATE_MODULE_APP,
                                              ENET_PKTSTATE_APP_WITH_FREEQ,
                                              ENET_PKTSTATE_APP_WITH_DRIVER);

                        /* Submit TX packet */
                        status = EnetDma_submitTxPkt(gEnetFOTA_LLDObj.hTxCh,
                                                     pktInfoTx);

                        currPktCnt += 1U;

                        /* Check for finish */
                        if(currPktCnt >= totalPktCnt)
                        {
                            finished = true;
                        }
                    }
                    else
                    {
                        /* Check if the packet number received is for previous packet, indicating that
                         * the ACK (or this is second iteration of same packet) got lost in transit
                         * and hence the host re-transmitted it due to timeout */
                        DebugP_assert(gEnetFOTA_MetaObj.appPktNum == currPktCnt-1);

                        uint32_t tempCurrPktCnt = currPktCnt-1;

                        /* Fill the TX payload with prev seq number and ACK code */
                        memcpy(&gEnetFOTA_MetaObj.txPayload[0], &tempCurrPktCnt, sizeof(tempCurrPktCnt));
                        memcpy(&gEnetFOTA_MetaObj.txPayload[sizeof(tempCurrPktCnt)], &EnetFOTA_Ack, sizeof(EnetFOTA_Ack));

                        /* Get free tx packet from queue */
                        pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
                        while (pktInfoTx == NULL)
                        {
                            txFreeCnt = EnetFOTA_retrieveFreeTxPkts();
                            if (txFreeCnt == 0)
                            {
                                SemaphoreP_pend(&gEnetFOTA_LLDObj.txSemObj, SystemP_WAIT_FOREVER);
                            }
                            pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
                        }

                        /* Init TX packet header */
                        EnetFOTA_initPkt(pktInfoTx, gEnetFOTA_MetaObj.txPayload);

                        EnetDma_checkPktState(&pktInfoTx->pktState,
                                              ENET_PKTSTATE_MODULE_APP,
                                              ENET_PKTSTATE_APP_WITH_FREEQ,
                                              ENET_PKTSTATE_APP_WITH_DRIVER);

                        /* Submit TX packet */
                        status = EnetDma_submitTxPkt(gEnetFOTA_LLDObj.hTxCh,
                                                     pktInfoTx);
                    }
                }

                /* Release the received packet */
                EnetQueue_enq(&gEnetFOTA_LLDObj.rxFreeQ, &pktInfoRx->node);
                pktInfoRx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.rxReadyQ);

                if(finished)
                {
                    status = ENET_SOK;
                    break;
                }
            }

            /*Submit now processed buffers */
            EnetAppUtils_validatePacketState(&gEnetFOTA_LLDObj.rxFreeQ,
                                                 ENET_PKTSTATE_APP_WITH_FREEQ,
                                                 ENET_PKTSTATE_APP_WITH_DRIVER);
            EnetDma_submitRxPktQ(gEnetFOTA_LLDObj.hRxCh,
                                     &gEnetFOTA_LLDObj.rxFreeQ);
        }
    }
    EnetAppUtils_print("[ ENETFOTA ] Status:%d\r\n", status);
    return status;
}

int32_t EnetFOTA_txFlashResp(Bootloader_UniflashResponseHeader respHeader)
{
    EnetDma_Pkt *pktInfoTx;
    uint32_t txFreeCnt;
    int32_t status = ENET_SOK;

    /* Fill the TX payload with seq number and ACK code */
    memcpy(&gEnetFOTA_MetaObj.txPayload[0], &respHeader, sizeof(respHeader));

    pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
    while (pktInfoTx == NULL)
    {
        txFreeCnt = EnetFOTA_retrieveFreeTxPkts();
        if (txFreeCnt == 0)
        {
            SemaphoreP_pend(&gEnetFOTA_LLDObj.txSemObj, SystemP_WAIT_FOREVER);
        }
        pktInfoTx = (EnetDma_Pkt *)EnetQueue_deq(&gEnetFOTA_LLDObj.txFreePktInfoQ);
    }

    /* Init TX packet header */
    EnetFOTA_initPkt(pktInfoTx, gEnetFOTA_MetaObj.txPayload);

    EnetDma_checkPktState(&pktInfoTx->pktState,
                          ENET_PKTSTATE_MODULE_APP,
                          ENET_PKTSTATE_APP_WITH_FREEQ,
                          ENET_PKTSTATE_APP_WITH_DRIVER);

    /* Submit TX packet */
    status = EnetDma_submitTxPkt(gEnetFOTA_LLDObj.hTxCh,
                                 pktInfoTx);
    return status;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static uint32_t EnetFOTA_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    int32_t status = 0;
    uint32_t txFreeQCnt = 0U;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any CPSW packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetFOTA_LLDObj.hTxCh, &txFreeQ);

    /* Push them into application object's free queue */
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_FREEQ);
            EnetQueue_enq(&gEnetFOTA_LLDObj.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }

    else
    {
        EnetAppUtils_print("[ ENETFOTA ] retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n",
                           status);
    }

    /* Return number of free packets retrieved */
    return txFreeQCnt;
}

static uint32_t EnetFOTA_receivePkts(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pktInfo;
    int32_t status = 0;
    uint32_t rxReadyCnt = 0U;

    EnetQueue_initQ(&rxReadyQ);

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetFOTA_LLDObj.hRxCh, &rxReadyQ);
    if (status == ENET_SOK)
    {
        rxReadyCnt = EnetQueue_getQCount(&rxReadyQ);

        /* Queue the received packet to rxReadyQ and pass new ones from rxFreeQ */
        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (pktInfo != NULL)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_DRIVER,
                                  ENET_PKTSTATE_APP_WITH_READYQ);

            EnetQueue_enq(&gEnetFOTA_LLDObj.rxReadyQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }
    }
    else
    {
        EnetAppUtils_print("[ ENETFOTA ] receivePkts() failed to retrieve pkts: %d\r\n", status);
    }

    return rxReadyCnt;
}

static int32_t EnetFOTA_setupCpswAle(void)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPortStateInArgs setPortStateInArgs;
    CpswAle_SetUcastEntryInArgs setUcastInArgs;
    uint32_t entryIdx;
    int32_t status = 0;

    /* ALE entry with "secure" bit cleared is required */
    setUcastInArgs.addr.vlanId  = 0U;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure  = false;
    setUcastInArgs.info.super   = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk   = false;
    EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], gEnetFOTA_LLDObj.hostMacAddr);
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(gEnetFOTA_LLDObj.hEnet, gEnetFOTA_LLDObj.coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms,status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ ENETFOTA ] Failed to add ucast entry: %d\r\n", status);
    }

    /* Set host port to 'forwarding' state */
    if (status == ENET_SOK)
    {
        setPortStateInArgs.portNum   = CPSW_ALE_HOST_PORT_NUM;
        setPortStateInArgs.portState = CPSW_ALE_PORTSTATE_FORWARD;
        ENET_IOCTL_SET_IN_ARGS(&prms, &setPortStateInArgs);

        ENET_IOCTL(gEnetFOTA_LLDObj.hEnet, gEnetFOTA_LLDObj.coreId, CPSW_ALE_IOCTL_SET_PORT_STATE, &prms,status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("[ ENETFOTA ] Failed to set ALE port state: %d\r\n", status);
        }
    }
    return status;
}


static void EnetFOTA_closeEnet(void)
{
    Enet_IoctlPrms prms;
    int32_t status = 0;

    for (uint32_t portIdx = 0; portIdx < gEnetFOTA_LLDObj.numMacPorts; portIdx++)
    {
        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &gEnetFOTA_LLDObj.macPort[portIdx]);

        ENET_IOCTL(gEnetFOTA_LLDObj.hEnet, gEnetFOTA_LLDObj.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms,status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("[ ENETFOTA ] Failed to close port %d link: %d\r\n", gEnetFOTA_LLDObj.macPort[portIdx], status);
        }
    }

}

static int32_t EnetFOTA_showAlivePhys(void)
{
    Enet_IoctlPrms prms;
    bool alive = false;
    int32_t status;

    for (uint32_t phyAdd = 0U; phyAdd < ENET_MDIO_PHY_CNT_MAX; phyAdd++)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAdd, &alive);

        ENET_IOCTL(gEnetFOTA_LLDObj.hEnet, gEnetFOTA_LLDObj.coreId, ENET_MDIO_IOCTL_IS_ALIVE, &prms,status);
        if (status == ENET_SOK)
        {
            if (alive == true)
            {
                EnetAppUtils_print("[ ENETFOTA ] PHY %u is alive\r\n", phyAdd);
            }
        }
        else
        {
            EnetAppUtils_print("[ ENETFOTA ] Failed to get PHY %u alive status: %d\r\n", phyAdd, status);
        }
    }

    return status;
}

static int32_t EnetFOTA_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    // wait for both the links to be up
    bool linked = false;

    while(!linked)
    {
        for(uint32_t portIdx = 0; portIdx < gEnetFOTA_LLDObj.numMacPorts; portIdx++)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetFOTA_LLDObj.macPort[portIdx], &linked);
            ENET_IOCTL(gEnetFOTA_LLDObj.hEnet, gEnetFOTA_LLDObj.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("[ ENETFOTA ]  Failed to get port link status: %d\r\n",  status);
                linked = false;
                break;
            }
            ClockP_sleep(2);
            if(linked == true)
            {
                goto linkupdone;
            }
        }
        if (status != ENET_SOK)
        {
            /* break if any port linkup has timedout */
            break;
        }
    }
    linkupdone:
        /* Sleep for 2 sec to complete host PC link up */
        ClockP_sleep(2U);
    return status;

}


static int32_t EnetFOTA_macMode2PhyMii(emac_mode macMode,
                                    EnetPhy_Mii *mii)
{
    int32_t status = ENET_SOK;

    switch (macMode)
    {
        case RMII:
            *mii = ENETPHY_MAC_MII_RMII;
            break;
        case RGMII:
            *mii = ENETPHY_MAC_MII_RGMII;
            break;
        default:
            status = ENET_EFAIL;
            EnetAppUtils_print("[ ENETFOTA ] Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }

    return status;
}

static void EnetFOTA_rxIsrFxn(void *appData)
{
    gEnetFOTA_LLDObj.rxIsrCount++;
    SemaphoreP_post(&gEnetFOTA_LLDObj.rxSemObj);
}

static void EnetFOTA_txIsrFxn(void *appData)
{
    gEnetFOTA_LLDObj.txIsrCount++;
    SemaphoreP_post(&gEnetFOTA_LLDObj.txSemObj);
}

void EnetApp_initEthFrameHdr(uint8_t *bufPtr, uint32_t *len)
{
    EthFrame *frame;

    frame = (EthFrame *)bufPtr;
    memcpy(frame->hdr.dstMac, enetAppAddrInfo.dstMac, ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    memcpy(frame->hdr.srcMac, &gEnetFOTA_LLDObj.hostMacAddr[0U], ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    frame->hdr.etherType = Enet_htons(ENET_LPBK_ETHERTYPE_IPV4);
    *len += sizeof(frame->hdr.etherType);
}


static bool EnetApp_verifyIPv4Checksum(uint16_t *header)
{
    uint32_t i;
    uint32_t sum = 0U;

    for(i = 0U; i < sizeof(EthAppIPv4Header)/2; i+=1)
    {
        sum += header[i];
    }

    /* Add the carries from top 16-bits */
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (sum == 0xFFFFU);
}

static uint16_t EnetApp_calcIPv4Checksum(uint8_t src_ip[IPV4_ADDR_LEN], uint8_t dest_ip[IPV4_ADDR_LEN])
{
    uint32_t sum = (uint16_t)((IPV4_HDR_VER_IHL << 8) | IPV4_HDR_TOS) + (uint16_t)(IPV4_HDR_TOTAL_PKT_LEN) + (uint16_t)(IPV4_HDR_IPID) + (uint16_t)(IPV4_HDR_FLAGFRAFOFFSET) + (uint16_t)((IPV4_HDR_TTL << 8) | IPV4_HDR_UDP);
    uint32_t i;

    /* Add source IP addr*/
    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)src_ip[i] << 8) | (uint32_t)src_ip[i+1]);
    }

    /* Add destination IP addr*/
    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)dest_ip[i] << 8) | (uint32_t)dest_ip[i+1]);
    }

    /* Add the carries from top 16-bits */
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    /* Return the one's complement of sum */
    return (uint16_t)((~sum) & 0xFFFF);
}

void EnetApp_initIPv4Hdr(uint8_t *bufPtr, uint32_t *len)
{
    EthAppIPv4Header *ipv4Hdr;

    ipv4Hdr = (EthAppIPv4Header *)bufPtr;
    ipv4Hdr->verIHL = IPV4_HDR_VER_IHL;
    ipv4Hdr->tos    = IPV4_HDR_TOS;
    ipv4Hdr->totalPktLen = Enet_htons(IPV4_HDR_TOTAL_PKT_LEN);
    ipv4Hdr->ipId = Enet_htons(IPV4_HDR_IPID);
    ipv4Hdr->flagFragOffset = Enet_htons(IPV4_HDR_FLAGFRAFOFFSET);
    ipv4Hdr->ttl = IPV4_HDR_TTL;
    ipv4Hdr->protocol = IPV4_HDR_UDP;
    ipv4Hdr->hdrChksum = Enet_htons(EnetApp_calcIPv4Checksum((uint8_t*)enetAppAddrInfo.srcIP, (uint8_t*)enetAppAddrInfo.dstIP));
    memcpy(&ipv4Hdr->srcIP,enetAppAddrInfo.srcIP,sizeof(ipv4Hdr->srcIP));
    memcpy(&ipv4Hdr->dstIP,enetAppAddrInfo.dstIP,sizeof(ipv4Hdr->dstIP));
    *len += sizeof(EthAppIPv4Header);
}

static bool EnetApp_verifyUDPChecksum(EthAppIPv4Header *ipv4Hdr, EthAppUDPHeader *udpHdr, uint8_t *data_payload)
{
    uint8_t *payload = data_payload;
    uint32_t sum = 0U;
    uint32_t i;
    uint32_t length = (uint32_t)Enet_ntohs(udpHdr->length);

    /* Add the pseudo-header */
    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)ipv4Hdr->srcIP[i] << 8) | (uint32_t)ipv4Hdr->srcIP[i+1]);
    }

    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)ipv4Hdr->dstIP[i] << 8) | (uint32_t)ipv4Hdr->dstIP[i+1]);
    }

    sum += ipv4Hdr->protocol;
    sum += Enet_ntohs(udpHdr->length);
    sum += Enet_ntohs(udpHdr->srcPort);
    sum += Enet_ntohs(udpHdr->dstPort);
    sum += Enet_ntohs(udpHdr->length);
    sum += Enet_ntohs(udpHdr->csum);

    /* Add payload */
    while (length > sizeof(EthAppUDPHeader)+1)
    {
        sum += ((uint32_t)*(payload) << 8) | (uint32_t)*(payload + 1);
        payload += 2;

        /* If 32-bit number going to overflow, add accumulated carries from top 16-bits */
        if (sum & 0x80000000)
        {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }

        length -= 2;
    }

    if (Enet_ntohs(udpHdr->length) & 1)
    {
        /* Add padding if the packet length is odd */
        sum += ((uint32_t)*(payload) << 8);
    }

    /* Add the carries from top 16-bits */
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (sum == 0xFFFFU);
}

static uint16_t EnetApp_calcUDPChecksum(uint8_t src_ip[IPV4_ADDR_LEN], uint16_t src_port, uint8_t dest_ip[IPV4_ADDR_LEN], uint16_t dest_port, uint16_t ip_protocol, uint16_t len, uint8_t *data_payload)
{
    uint8_t *payload = data_payload;
    uint32_t sum = 0U;
    uint32_t i;
    uint32_t length = (uint32_t)len;

    /* Add the pseudo-header */
    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)src_ip[i] << 8) | (uint32_t)src_ip[i+1]);
    }

    for(i = 0U; i < IPV4_ADDR_LEN; i+=2)
    {
        sum += (((uint32_t)dest_ip[i] << 8) | (uint32_t)dest_ip[i+1]);
    }

    sum += ip_protocol;
    sum += len;
    sum += src_port;
    sum += dest_port;
    sum += len;

    /* Add payload */
    while (length > sizeof(EthAppUDPHeader)+1)
    {
        sum += ((uint32_t)*(payload) << 8) | (uint32_t)*(payload + 1);
        payload += 2;

        /* If 32-bit number going to overflow, add accumulated carries from top 16-bits */
        if (sum & 0x80000000)
        {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }

        length -= 2;
    }

    if (len & 1)
    {
        /* Add padding if the packet length is odd */
        sum += ((uint32_t)*(payload) << 8);
    }

    /* Add the carries from top 16-bits */
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    /* Return the one's complement of sum */
    return (uint16_t)((~sum) & 0xFFFF);
}

static void EnetApp_initUDPHdr(uint8_t *bufPtr, uint32_t *len, uint8_t *payload)
{
    EthAppUDPHeader *udpHdr;

    udpHdr = (EthAppUDPHeader *)bufPtr;
    udpHdr->srcPort = Enet_htons(enetAppAddrInfo.srcPortUDP);
    udpHdr->dstPort = Enet_htons(enetAppAddrInfo.dstPortUDP);
    udpHdr->length = Enet_htons(UDP_PKT_LEN);
    udpHdr->csum = Enet_htons(EnetApp_calcUDPChecksum((uint8_t*)enetAppAddrInfo.srcIP,
                                                      enetAppAddrInfo.srcPortUDP,
                                                      (uint8_t*)enetAppAddrInfo.dstIP,
                                                      enetAppAddrInfo.dstPortUDP,
                                                      (uint16_t)IPV4_HDR_UDP,
                                                      (uint16_t)UDP_PKT_LEN,
                                                      payload));
    *len += sizeof(EthAppUDPHeader);
}

static void EnetFOTA_initPkt(EnetDma_Pkt *pPktInfo, uint8_t *udpPayload)
{
    uint32_t len;

    len = 0;
    EnetApp_initEthFrameHdr(&gEnetFOTA_MetaObj.appPktHeader[len], &len);
    EnetApp_initIPv4Hdr(&gEnetFOTA_MetaObj.appPktHeader[len], &len);
    EnetApp_initUDPHdr(&gEnetFOTA_MetaObj.appPktHeader[len], &len, &udpPayload[0]);
    pPktInfo->rxPortNum = ENET_MAC_PORT_2;
    pPktInfo->chkSumInfo = 0U;
    pPktInfo->sgList.list[0].segmentFilledLen = len;
    pPktInfo->appPriv    = &gEnetFOTA_LLDObj;
    pPktInfo->sgList.numScatterSegments = 2;
    pPktInfo->sgList.list[1].bufPtr = udpPayload;
    pPktInfo->sgList.list[1].segmentFilledLen = IPV4_HDR_TOTAL_PKT_LEN - (sizeof(EthAppUDPHeader) + sizeof(EthAppIPv4Header));
    pPktInfo->sgList.list[0].bufPtr = &gEnetFOTA_MetaObj.appPktHeader[0];
    pPktInfo->sgList.list[0].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[0].bufPtr, len, CacheP_TYPE_ALLD);
    pPktInfo->sgList.list[1].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[1].bufPtr, (IPV4_HDR_TOTAL_PKT_LEN - (sizeof(EthAppUDPHeader) + sizeof(EthAppIPv4Header))), CacheP_TYPE_ALLD);
}

static void EnetFOTA_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetFOTA_LLDObj.txFreePktInfoQ);

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_TX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetFOTA_LLDObj,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetDma_initPktInfo(pPktInfo);

        EnetQueue_enq(&gEnetFOTA_LLDObj.txFreePktInfoQ, &pPktInfo->node);
    }

    //EnetAppUtils_print("[ ENETFOTA ] initQs() txFreePktInfoQ initialized with %d pkts\r\n", EnetQueue_getQCount(&gEnetFOTA_LLDObj.txFreePktInfoQ));
}

static void EnetFOTA_initRxReadyPktQ(void)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt *pPktInfo;
    int32_t status;
    uint32_t i;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&gEnetFOTA_LLDObj.rxFreeQ);
    EnetQueue_initQ(&gEnetFOTA_LLDObj.rxReadyQ);
    EnetQueue_initQ(&rxReadyQ);

    for (i = 0U; i < ENET_SYSCFG_TOTAL_NUM_RX_PKT; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetFOTA_LLDObj,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);
        EnetQueue_enq(&gEnetFOTA_LLDObj.rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any CPSW packets which are ready */
    status = EnetDma_retrieveRxPktQ(gEnetFOTA_LLDObj.hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);
    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&gEnetFOTA_LLDObj.rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(gEnetFOTA_LLDObj.hRxCh,
                         &gEnetFOTA_LLDObj.rxFreeQ);

    /* Assert here as during init no. of DMA descriptors should be equal to
     * no. of free Ethernet buffers available with app */

    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetFOTA_LLDObj.rxFreeQ));
}

static int32_t EnetFOTA_openDma(void)
{
    int32_t status = ENET_SOK;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     txInArgs;
        EnetApp_GetTxDmaHandleOutArgs  txChInfo;

        txInArgs.cbArg   = &gEnetFOTA_LLDObj;
        txInArgs.notifyCb = EnetFOTA_txIsrFxn;

        EnetApp_getTxDmaHandle(ENET_DMA_TX_CH0,
                               &txInArgs,
                               &txChInfo);

        gEnetFOTA_LLDObj.txChNum = txChInfo.txChNum;
        gEnetFOTA_LLDObj.hTxCh   = txChInfo.hTxCh;

        gEnetFOTA_LLDObj.txIsrCount = 0;
        EnetFOTA_initTxFreePktQ();

        if (NULL != gEnetFOTA_LLDObj.hTxCh)
        {
            status = ENET_SOK;
            if (ENET_SOK != status)
            {
                EnetAppUtils_print("[ ENETFOTA ] EnetUdma_startTxCh() failed: %d\r\n", status);
                status = ENET_EFAIL;
            }
        }
        else
        {
            EnetAppUtils_print("[ ENETFOTA ] EnetDma_openTxCh() failed to open: %d\r\n",
                               status);
            status = ENET_EFAIL;
        }
    }

    /* Open the CPSW RX flow  */
    if (status == ENET_SOK)
    {
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        EnetApp_GetDmaHandleInArgs     rxInArgs;

        rxInArgs.notifyCb = EnetFOTA_rxIsrFxn;
        rxInArgs.cbArg   = &gEnetFOTA_LLDObj;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                              &rxInArgs,
                              &rxChInfo);

        gEnetFOTA_LLDObj.rxFlowStartIdx = rxChInfo.rxFlowStartIdx;
        gEnetFOTA_LLDObj.rxFlowIdx = rxChInfo.rxFlowIdx;
        gEnetFOTA_LLDObj.hRxCh  = rxChInfo.hRxCh;

        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);

        if (NULL == gEnetFOTA_LLDObj.hRxCh)
        {
            EnetAppUtils_print("[ ENETFOTA ] EnetDma_openRxCh() failed to open: %d\r\n",
                               status);
            EnetAppUtils_assert(NULL != gEnetFOTA_LLDObj.hRxCh);
        }
        else
        {
            EnetAppUtils_assert(rxChInfo.numValidMacAddress > 0);
            EnetUtils_copyMacAddr(gEnetFOTA_LLDObj.hostMacAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
            EnetAppUtils_print("[ ENETFOTA ] EVM MAC address: ");
            EnetAppUtils_printMacAddr(gEnetFOTA_LLDObj.hostMacAddr);
            gEnetFOTA_LLDObj.rxIsrCount = 0;
            /* Submit all ready RX buffers to DMA.*/
            EnetFOTA_initRxReadyPktQ();
        }
    }

    return status;
}

static void EnetFOTA_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);


    /* There should not be any ready packet */
    EnetAppUtils_assert(0U == EnetQueue_getQCount(&gEnetFOTA_LLDObj.rxReadyQ));

    /* Close RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetFOTA_LLDObj.hEnet,
                       gEnetFOTA_LLDObj.coreKey,
                       gEnetFOTA_LLDObj.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetFOTA_LLDObj.hEnet,
                       gEnetFOTA_LLDObj.coreKey,
                       gEnetFOTA_LLDObj.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetFOTA_LLDObj.rxFreeQ);
    EnetAppUtils_freePktInfoQ(&gEnetFOTA_LLDObj.txFreePktInfoQ);

}

static void EnetFOTA_printFrame(EthFrame *frame, EthAppIPv4Header *ipv4Frame, EthAppUDPHeader *udpFrame, uint32_t dataLen)
{
    uint32_t i;
    EnetAppUtils_print("[ ENETFOTA ] ============================================\r\n");
    EnetAppUtils_print("[ ENETFOTA ] FRAME HEADER\r\n");
    EnetAppUtils_print("[ ENETFOTA ] ------------\r\n");

    /* ETH Frame Header */
    EnetAppUtils_print("[ ENETFOTA ] Dst addr : ");
    EnetAppUtils_printMacAddr(&frame->hdr.dstMac[0]);
    EnetAppUtils_print("[ ENETFOTA ] Src addr : ");
    EnetAppUtils_printMacAddr(&frame->hdr.srcMac[0]);
    EnetAppUtils_print("[ ENETFOTA ] EtherType: 0x%04x\r\n", Enet_ntohs(frame->hdr.etherType) & 0xFFFFU);

    /* IPv4 Frame Header */
    EnetAppUtils_print("[ ENETFOTA ] Version : 0x%02x\r\n", ipv4Frame->verIHL & 0xFFU);
    EnetAppUtils_print("[ ENETFOTA ] TOS : 0x%02x\r\n", ipv4Frame->tos & 0xFFU);
    EnetAppUtils_print("[ ENETFOTA ] Total Packet Len : 0x%04x (%d)\r\n",
                       Enet_htons(ipv4Frame->totalPktLen) & 0xFFFFU,
                       Enet_htons(ipv4Frame->totalPktLen) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] IP ID : 0x%04x\r\n", Enet_htons(ipv4Frame->ipId) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] Flag/Fragmentation Offset : 0x%04x\r\n", Enet_htons(ipv4Frame->flagFragOffset) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] TTL : 0x%02x\r\n", ipv4Frame->ttl & 0xFFU);
    EnetAppUtils_print("[ ENETFOTA ] Protocol : 0x%02x\r\n", ipv4Frame->protocol & 0xFFU);
    EnetAppUtils_print("[ ENETFOTA ] Checksum : 0x%04x\r\n", Enet_htons(ipv4Frame->hdrChksum) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] Src IP addr : %d.%d.%d.%d\r\n",
                       ipv4Frame->srcIP[0] & 0xFF,
                       ipv4Frame->srcIP[1] & 0xFF,
                       ipv4Frame->srcIP[2] & 0xFF,
                       ipv4Frame->srcIP[3] & 0xFF);
    EnetAppUtils_print("[ ENETFOTA ] Dest IP addr : %d.%d.%d.%d\r\n",
                       ipv4Frame->dstIP[0] & 0xFF,
                       ipv4Frame->dstIP[1] & 0xFF,
                       ipv4Frame->dstIP[2] & 0xFF,
                       ipv4Frame->dstIP[3] & 0xFF);

    /* UDP Frame Header */
    EnetAppUtils_print("[ ENETFOTA ] Source Port : 0x%04x (%d)\r\n",
                       Enet_htons(udpFrame->srcPort) & 0xFFFFU,
                       Enet_htons(udpFrame->srcPort) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] Destin Port : 0x%04x (%d)\r\n",
                       Enet_htons(udpFrame->dstPort) & 0xFFFFU,
                       Enet_htons(udpFrame->dstPort) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] Length : 0x%04x (%d)\r\n",
                       Enet_htons(udpFrame->length) & 0xFFFFU,
                       Enet_htons(udpFrame->length) & 0xFFFFU);
    EnetAppUtils_print("[ ENETFOTA ] Checksum : 0x%04x\r\n", Enet_htons(udpFrame->csum) & 0xFFFFU);

    EnetAppUtils_print("[ ENETFOTA ] \r\n-----------------------------\r\n");

    /* Payload */
    EnetAppUtils_print("[ ENETFOTA ] DATA PAYLOAD\r\n");
    EnetAppUtils_print("[ ENETFOTA ] ------------\r\n");

    for (i = 0; i < dataLen; i++)
    {
        EnetAppUtils_print("[ ENETFOTA ] 0x%02x ", gEnetFOTA_MetaObj.appPktData[i]);
        if (i && (((i + 1) % OCTETS_PER_ROW) == 0))
        {
            EnetAppUtils_print("[ ENETFOTA ] \r\n");
        }
    }

    if (dataLen && ((dataLen % OCTETS_PER_ROW) != 0))
    {
        EnetAppUtils_print("[ ENETFOTA ] \r\n");
    }

    EnetAppUtils_print("[ ENETFOTA ] \r\n");
}

static bool EnetFOTA_parseFrame(EthFrame *frame, uint32_t dataLen)
{
    uint8_t *payload;
    bool matchingPkt = false;

    payload = frame->payload;

    EthAppIPv4Header *ipv4Frame = (EthAppIPv4Header *) payload;
    EthAppUDPHeader *udpFrame = (EthAppUDPHeader *) &payload[sizeof(EthAppIPv4Header)];
    memset(&gEnetFOTA_MetaObj.appPktData[0], 0U, ENETFOTA_PKT_MAX_SIZE);

    /* Match IPv4 Address, Ethertype, UDP, Port number, IPv4 and UDP checksums, Magic Number */
    if(memcmp(&ipv4Frame->dstIP[0], &enetAppAddrInfo.srcIP[0], sizeof(ipv4Frame->dstIP)) == 0)
    {
        if (frame->hdr.etherType == Enet_htons(IPV4_ETHERTYPE))
        {
            if (ipv4Frame->protocol == IPV4_HDR_UDP)
            {
                if (udpFrame->srcPort == Enet_htons(enetAppAddrInfo.srcPortUDP))
                {
                    if (EnetApp_verifyIPv4Checksum((uint16_t*)ipv4Frame))
                    {
                        if (EnetApp_verifyUDPChecksum(ipv4Frame, udpFrame, &payload[sizeof(EthAppIPv4Header)+sizeof(EthAppUDPHeader)]))
                        {

                            if(memcmp(&payload[sizeof(EthAppIPv4Header)+sizeof(EthAppUDPHeader)], &EnetFOTA_MagicNum, MGC_NUM_SIZE) == 0)
                            {
                                memcpy(&gEnetFOTA_MetaObj.appPktData[0], &payload[sizeof(EthAppIPv4Header)+sizeof(EthAppUDPHeader)], sizeof(gEnetFOTA_MetaObj.appPktData[0])*dataLen);
                                matchingPkt = true;
                            }
                        }
                    }
                }
            }
        }
    }

    if(matchingPkt)
    {
        /* Copy the packet number from the payload to the global packet number */
        memcpy(&(gEnetFOTA_MetaObj.appPktNum), &gEnetFOTA_MetaObj.appPktData[MGC_NUM_SIZE], SEQ_NUM_SIZE);

        volatile bool printFrame = false; /* Use in debugging sessions to print appimage packets */
        if (printFrame)
        {
            EnetFOTA_printFrame(frame, ipv4Frame, udpFrame, dataLen);
        }
    }

    return matchingPkt;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;
// Check if needed

//    EnetCpdma_Cfg *dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;
//
//    dmaCfg->rxInterruptPerMSec = 8;
//    dmaCfg->txInterruptPerMSec = 2;

    /* Peripheral config */
    cpswCfg->vlanCfg.vlanAware = false;

    /* Host port config */
    hostPortCfg->removeCrc      = true;
    hostPortCfg->padShortPacket = true;
    hostPortCfg->passCrcErrors  = true;
    hostPortCfg->rxMtu          = 1518;

    /* ALE config */
    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = false;
    aleCfg->vlanCfg.cpswVlanAwareMode          = false;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;

    /* CPTS config */
    /* Note: Timestamping and MAC are not supported together because of
     * IP limitation, so disabling timestamping for this application */
    cptsCfg->hostRxTsEn = false;
}