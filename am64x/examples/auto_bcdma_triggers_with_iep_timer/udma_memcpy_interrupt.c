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

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example performs UDMA block copy transfer using Type 15 Transfer Record (TR15)
 * using Transfer Record Packet Descriptor (TRPD) in interrupt mode.
 *
 * The application opens and configures a BCDMA channel using SysConfig.
 * It also configures the interrupt mode of operation through the SysConfig
 * which ensures that all required interrupt configuration are done.
 * The callback function App_udmaEventCb is registered via SysConfig.
 *
 * Then the application prepares a TRPD for a 1D transfer from source to
 * destination buffer, submits the request to DMA, waits for the DMA to complete
 * by waiting on a semaphore which is posted in the callback function.
 *
 * Once the transfer it completes, it does cache operation for data coherency
 * and compares the source and destination buffers for any data mismatch.
 *
 */


#define TICK_TIME_8  3000UL
#define TICK_TIME_9  6000UL
#define TICK_TIME_10 9000UL
#define TICK_TIME_11 12000UL
#define TICK_TIME_12 15000UL
#define TICK_TIME_13 18000UL
#define TICK_TIME_14 21000UL
//#define TICK_TIME_15 24000UL

#define TICK_TIME_15 200000000


void IEP0_Init(void);
void CMP_router_Init(void);

void configure_router( uint16_t src_index,uint16_t dst_index);

#define ICSSG_SA_MX_REG_ADDR        (CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE + 0x40)

#define IEP0  0U
#define IEP1  1U

#define IEP_TIMER        IEP1


void configure_intaggrL2G(uint32_t localEvent, uint32_t globalEvent);
static uint32_t tempCnt = 0U;

/* Number of bytes to do memcpy */
#define UDMA_TEST_NUM_BYTES             (128U)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Application Buffers */
uint8_t gUdmaTestSrcBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTestDestBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gUdmaTestDoneSem;

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);
static void App_udmaInitBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);
static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);
Udma_ChHandle   chHandle;
uint8_t        *trpdMem = &gUdmaTestTrpdMem[0U];
extern void CMP_router_Init(void);
HwiP_Object         gGpioHwiObject;
void Utils_setupPdmaL2G(uint32_t localEventIndex, uint32_t globalEvent);


void *udma_memcpy_interrupt_main(void *args)
{
    int32_t         retVal = UDMA_SOK, status;
    uint8_t        *srcBuf = &gUdmaTestSrcBuf[0U];
    uint8_t        *destBuf = &gUdmaTestDestBuf[0U];
    uint32_t        length = UDMA_TEST_NUM_BYTES;
    uint32_t        globalEvent0;
    uint64_t        trpdMemPhy;
    uint32_t  count = 0;


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    IEP0_Init();

    chHandle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */
    DebugP_log("[UDMA] Memcpy application started ...\r\n");

    status = SemaphoreP_constructBinary(&gUdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);


    /* Init buffers and TR packet descriptor */
    App_udmaInitBuf(srcBuf, destBuf, length);
    App_udmaTrpdInit(chHandle, trpdMem, destBuf, srcBuf, length);



    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);

    /* Submit TRPD to channel */
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);


    /* Channel enable */
    retVal = Udma_chEnable(chHandle);
    DebugP_assert(UDMA_SOK == retVal);


    globalEvent0 = Udma_chGetTriggerEvent(chHandle, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);



   CMP_router_Init();
   configure_intaggrL2G(7U,globalEvent0);
   CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE,0x11);     ; //Start IEP timer

    while(1)
    {

        uint32_t        IEP_status;
        IEP_status = CSL_REG32_RD(CSL_PRU_ICSSG0_IEP1_BASE+0x74UL); //Clear compare status Register


        if( (IEP_status >> 15U) == 1U)
        {
            count = count + 1;

            CSL_REG32_WR( CSL_PRU_ICSSG0_IEP1_BASE+0x74UL, ( (1U << 15U) | (1U<< 1U) )  );
            App_udmaCompareBuf(srcBuf,destBuf,length);
            DebugP_log("Data Transferring for every 1sec based on IEP timer !!: %d\r\n", count);
        }



    }


    /* Channel disable */
    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    SemaphoreP_destruct(&gUdmaTestDoneSem);
    DebugP_log("All tests have passed!!\r\n");
    Board_driversClose();
    Drivers_close();

    return NULL;
}




static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length)
{
    CSL_UdmapTR15  *pTr;
    uint32_t        cqRingNum = Udma_chGetCqRingNum(chHandle);




    /* Make TRPD with TR15 TR type */
    UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);
    CSL_udmapCppi5TrSetReload((CSL_UdmapCppi5TRPD *)trpdMem, 0x1FFU, 0U);



    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = length;
    pTr->icnt1    = 1U;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = 0U;
    pTr->dim3     = 0U;
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;


    pTr->dicnt0   = length;
    pTr->dicnt1   = 1U;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = 0U;
    pTr->ddim3    = 0U;
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);


    /* Perform cache writeback */
    CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    return;
}

static void App_udmaInitBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length)
{
    uint32_t        i;

    for(i = 0U; i < length; i++)
    {
        srcBuf[i] = i;
        destBuf[i] = 0xA5U;
    }
    /* Writeback source and destination buffer */
    CacheP_wb(srcBuf, length, CacheP_TYPE_ALLD);
    CacheP_wb(destBuf, length, CacheP_TYPE_ALLD);

    return;
}


static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length)
{
    uint32_t        i, flag=0;

    /* Invalidate destination buffer */
    CacheP_inv(destBuf, length, CacheP_TYPE_ALLD);
    for(i = 0U; i < length; i++)
    {
        if(srcBuf[i] != destBuf[i])
        {
            flag = 1U;
            break;
        }

    }

    for(i = 0U; i < length; i++)
    {
        destBuf[i] = 0U;

    }

    if(flag)
      {
        tempCnt++;
        DebugP_log("Error count %d: tempCnt");

      }


    return;
}


void configure_intaggrL2G(uint32_t localEvent, uint32_t globalEvent)
{
    uint64_t eventRegOffset = CSL_DMASS0_INTAGGR_L2G_BASE + (localEvent * 0x20U);
    CSL_REG32_WR(eventRegOffset, ( (0U << 31U) | (globalEvent & 0xFFFFU) ) );
}


void IEP0_Init(void)
{


#if (IEP_TIMER ==       IEP0)

    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE,0x10); //reset iep0 timer
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x10UL,0xffffffff); //Reset Low Count Register
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x14UL,0xffffffff); //Reset High Count Register
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x4UL,0x01); //Clear overflow status register
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x74UL,0xffffffff); //Clear compare status Register

    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x78UL,TICK_TIME_15);

    /*CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xC0UL,TICK_TIME_8);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xC8UL,TICK_TIME_9);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xD0UL,TICK_TIME_10);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xD8UL,TICK_TIME_11);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xE0UL,TICK_TIME_12);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xE8UL,TICK_TIME_13);
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xF0UL,TICK_TIME_14);*/
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0xF8UL,TICK_TIME_15);

    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x70UL,0x10203); //Shadow disabled + Enabled CMP0  + Reset counter After reaching CMP0 Counter
    CSL_REG32_WR(CSL_PRU_ICSSG0_IEP0_BASE+0x74UL,0xffffffff); //Clear compare status Register

    /* set bit to enable CMP auto-clear mode */
     //CSL_REG32_WR(ICSSG_SA_MX_REG_ADDR, 1U << 16U);
#else
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE,0x10); //reset iep0 timer
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x10UL,0xffffffff); //Reset Low Count Register
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x14UL,0xffffffff); //Reset High Count Register
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x4UL,0x01); //Clear overflow status register
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x74UL,0xffffffff); //Clear compare status Register

     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x78UL,TICK_TIME_15);

     /*CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xC0UL,TICK_TIME_8);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xC8UL,TICK_TIME_9);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xD0UL,TICK_TIME_10);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xD8UL,TICK_TIME_11);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xE0UL,TICK_TIME_12);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xE8UL,TICK_TIME_13);
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xF0UL,TICK_TIME_14);*/
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0xF8UL,TICK_TIME_15);

     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x70UL,0x10203); //Shadow disabled + Enabled CMP0  + Reset counter After reaching CMP0 Counter
     CSL_REG32_WR(CSL_PRU_ICSSG0_IEP1_BASE+0x74UL,0xffffffff); //Clear compare status Register

#endif
}


void CMP_router_Init(void)
{

    configure_router(CSLR_CMP_EVENT_INTROUTER0_IN_PRU_ICSSG0_PR1_IEP1_CMP_INTR_REQ_15,(CSLR_DMASS0_INTAGGR_0_INTAGGR_LEVI_PEND_CMP_EVENT_INTROUTER0_OUTP_39 + 32U) );


}


void configure_router( uint16_t src_index,uint16_t dst_index)
{
    int32_t                             retVal;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event          |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
    //rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = TISCI_DEV_CMP_EVENT_INTROUTER0;//TISCI_DEV_PRU_ICSSG0;
    rmIrqReq.src_index              = src_index;
    rmIrqReq.dst_id                 = TISCI_DEV_CMP_EVENT_INTROUTER0;//TISCI_DEV_DMASS0_INTAGGR_0;
    rmIrqReq.dst_host_irq           = dst_index;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    //retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    retVal = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    if(0 != retVal)
    {
        DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        DebugP_assert(FALSE);
    }

    return;
}
