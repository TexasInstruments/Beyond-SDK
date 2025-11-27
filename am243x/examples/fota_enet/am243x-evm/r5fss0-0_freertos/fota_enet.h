/*
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated
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
 * \file  fota_enet.h
 *
 * \brief This is the common header file of fota_enet application.
 */

#ifndef _FOTA_ENET_H_
#define _FOTA_ENET_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <networking/enet/core/utils/include/enet_apputils.h>
#include <networking/enet/core/utils/include/enet_appmemutils.h>
#include <networking/enet/core/utils/include/enet_appmemutils_cfg.h>

#include "ti_board_config.h"
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <ti_enet_config.h>

#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void Board_cpswMuxSel(void);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

// Change the PC MAC Address to the MAC Address of the corresponding Ethernet Adapter
/* Host PC MAC Address */
 #define ENET_HOST_PC_MAC_ADDRESS        {0xF8, 0xED, 0xFC, 0x59, 0x8A, 0xEB} // TP-link

/* Port */
#define ENET_PORT                       5001
/* EVM IP ADDRESS */
#define ENET_SOURCE_IP_ADDRESS          { 192U, 168U, 0U, 195U }
/* Host PC IP Address */
#define ENET_DESTINATION_IP_ADDRESS     { 192U, 168U, 0U, 193U }

#define ENET_MTU_SIZE                   (1514U)
#define ENET_INSTANCE_ID                (0U)
#define ENET_MAC_PORT                   (ENET_MAC_PORT_2)
#define ENET_MAC_MODE                   (RGMII)
#define ENET_TYPE                       (ENET_CPSW_3G)
#define ENET_BOARD_ID                   (ENETBOARD_CPB_ID)

#define IPV4_HDR_VER_IHL                ((0x4 << 4) | 0x5)
#define IPV4_HDR_TOS                    (0x00)
#define IPV4_HDR_TOTAL_PKT_LEN          (36U)
#define IPV4_HDR_IPID                   (0x28)
#define IPV4_HDR_FLAGFRAFOFFSET         (0x0000)
#define IPV4_HDR_TTL                    (0xFF)
#define IPV4_HDR_UDPLITE                (0x88)
#define IPV4_HDR_UDP                    (0x11)
#define IPV4_ADDR_LEN                   (4U)
#define IPV4_ETHERTYPE                  (0x0800)

#define UDP_PKT_LEN                     (IPV4_HDR_TOTAL_PKT_LEN - IPV4_HDR_SIZE)

#define ETH_HDR_SIZE                    (14U)
#define IPV4_HDR_SIZE                   (20U)
#define UDP_HDR_SIZE                    (8U)
#define SEQ_NUM_SIZE                    (4U)
#define ACK_CODE_SIZE                   (4U)
#define MGC_NUM_SIZE                    (4U)

#define ENETFOTA_PKT_HDR_SIZE            (ENET_UTILS_ALIGN((UDP_HDR_SIZE + IPV4_HDR_SIZE + ETH_HDR_SIZE),128))
#define ENETFOTA_PKT_MAX_SIZE            (ENET_UTILS_ALIGN((ENET_MTU_SIZE - UDP_HDR_SIZE - IPV4_HDR_SIZE - ETH_HDR_SIZE),128))
#define ENETFOTA_TX_PAYLOAD_SIZE         (ENET_UTILS_ALIGN((SEQ_NUM_SIZE + ACK_CODE_SIZE),8))

#define ENETFOTA_HEADER_MGC_NUMBER       (0x05B1C00D)
#define ENETFOTA_HEADER_ACK              (0x05B10ACD)

#define MAX_FILE_SIZE        (10500000) /* This has to match the size of MSRAM_2 in linker.cmd */

#define PERIODIC_TICK_MS                (100U) /* 100-ms periodic tick */

#define COUNTING_SEM_COUNT              (10U) /* Counting Semaphore count */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef enum EnetFOTA_type_e
{
    /* PHY loopback(internal) */
    ENET_LOOPBACK_TYPE_MAC  = 0,
    ENET_LOOPBACK_TYPE_PHY  = 1,
    /* No loopback. trasmit packets to network */
    ENET_LOOPBACK_TYPE_NONE = 2
} EnetFOTA_type;

typedef struct EnetFOTA_LLDObj_s
{
    /* Enet driver */
    Enet_Handle hEnet;
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    Enet_MacPort macPort[ENET_SYSCFG_MAX_MAC_PORTS];
    uint8_t numMacPorts;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];

    /* Tx, Rx Packet Queues */
    uint32_t rxChNum;
    EnetDma_RxChHandle hRxCh;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ rxReadyQ;
    EnetDma_TxChHandle hTxCh;
    EnetDma_PktQ txFreePktInfoQ;
    uint32_t txChNum;

    /* RX start flow index */
    uint32_t rxFlowStartIdx;
    /* RX flow index */
    uint32_t rxFlowIdx;

    /* Periodic tick */
    ClockP_Object timerObj;
    SemaphoreP_Object timerSemObj;

    /* Packet transmission */
    SemaphoreP_Object txSemObj;
    uint32_t totalTxCnt;

    /* Packet reception */
    SemaphoreP_Object rxSemObj;
    uint32_t totalRxCnt;

    /* ISR functions */
    uint32_t rxIsrCount;
    uint32_t txIsrCount;

    EnetFOTA_type testLoopBackType;
    bool printFrame;        /* Print received Ethernet frames? */

    emac_mode macMode;
    uint32_t boardId;
    EventP_Object appEvents;
} EnetFOTA_LLDObj;

typedef struct EnetFOTA_MetaObj_s
{
    /* Transfer-related */
    uint32_t appPktNum;
    uint8_t appPktData[ENETFOTA_PKT_MAX_SIZE] __attribute__ ((aligned(128)));
    uint8_t appPktHeader[ENETFOTA_PKT_HDR_SIZE] __attribute__ ((aligned(128)));
    uint8_t txPayload[ENETFOTA_TX_PAYLOAD_SIZE] __attribute__ ((aligned(8)));
} EnetFOTA_MetaObj;

typedef struct
{
    uint8_t verIHL;
    uint8_t tos;
    uint16_t totalPktLen;
    uint16_t ipId;
    uint16_t flagFragOffset;
    uint8_t  ttl;
    uint8_t protocol;
    uint16_t hdrChksum;
    uint8_t  srcIP[IPV4_ADDR_LEN];
    uint8_t  dstIP[IPV4_ADDR_LEN];
} __attribute__ ((packed)) EthAppIPv4Header;

typedef struct
{
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t length;
    uint16_t csum;
} __attribute__ ((packed)) EthAppUDPHeader;

typedef struct EnetFOTA_AddrInfo_s
{
    uint8_t dstMac[ENET_MAC_ADDR_LEN];
    uint8_t srcIP[IPV4_ADDR_LEN];
    uint8_t dstIP[IPV4_ADDR_LEN];
    uint16_t srcPortUDP;
    uint16_t dstPortUDP;
} EnetFOTA_AddrInfo;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetFOTA_setup(void);

void EnetFOTA_destruct(void);

int32_t EnetFOTA_transferAppimage(void);

int32_t EnetFOTA_txFlashResp(Bootloader_UniflashResponseHeader respHeader);

int32_t EnetFOTA_TransferStart(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet test object declaration */
extern EnetFOTA_LLDObj gEnetFOTA_LLDObj;
extern EnetFOTA_MetaObj gEnetFOTA_MetaObj;
extern uint8_t gFlashFileBuf[MAX_FILE_SIZE] __attribute__((aligned(128))) __attribute__((section(".ddr_memory")));
extern uint32_t gFlashFileSize;

#ifdef __cplusplus
}
#endif

#endif /* _FOTA_ENET_H_ */
