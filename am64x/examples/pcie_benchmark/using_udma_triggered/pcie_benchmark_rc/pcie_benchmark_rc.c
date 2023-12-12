/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>
#include <drivers/udma.h>

/******** Buffer *******/

#define MIN_PACKET_SIZE 40
#define MAX_PACKET_SIZE 66000 //0x4000

/* NUmber of Iterations */
#define NUM_ITER  1 // max value 10 (BUF_SIZE/MAX_PACKET_SIZE)

/* Semaphore to indicate CPU transfer completion */
static SemaphoreP_Object gBufTransDoneSem;
/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gBufTransUDMADoneSem;

/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))

/* This example is applicable for even (1.2,4,8,16) count of TRPDs only */
#define NUM_CHANNEL 16 //it should be same as CONFIG_UDMA0_NUM_BLKCOPY_CH

#define channel_1           0
#define channel_2           1

#if NUM_CHANNEL > 2
    #define NUM_CHANNEL_4
    #define channel_3       2
    #define channel_4       3
#endif

#if NUM_CHANNEL > 4
    #define NUM_CHANNEL_8
    #define channel_5       4
    #define channel_6       5
    #define channel_7       6
    #define channel_8       7

#endif


#if NUM_CHANNEL > 8
    #define NUM_CHANNEL_16
    #define channel_9       8
    #define channel_10      9
    #define channel_11      10
    #define channel_12      11
    #define channel_13      12
    #define channel_14      13
    #define channel_15      14
    #define channel_16      15
#endif
/******** Buffer *******/
#define BUF_SIZE (0x4000000u / NUM_CHANNEL) //Total of  All channels should be 64MB

uint8_t ddr_buf[NUM_CHANNEL][BUF_SIZE]  __attribute__ ((section (".buffer"), aligned (BUF_SIZE)));

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[NUM_CHANNEL][UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

Udma_DrvHandle      drvHandle = &gUdmaDrvObj[0];
Udma_EventPrms      eventPrms;

Udma_EventObject    gUdmaCqEventObj[NUM_CHANNEL];

Udma_ChHandle       chHandle[NUM_CHANNEL];
uint8_t     *trpdMem[NUM_CHANNEL];
uint64_t    trpdMemPhy[NUM_CHANNEL];

static int32_t completed_channel_transfer[NUM_CHANNEL]; //completed transfer on channel 1-4

//Below region (pcie_BufAddr_1 + pcie_BufAddr_2) should be < 0x70000000 (outbound region fixed sysconfig)
void *      pcie_BufAddr[NUM_CHANNEL];


void PCIE_udmaEventDmaCb(Udma_EventHandle eventHandle, uint32_t eventType,
                              void *appData);
void Pcie_cpu_enable_legacy_isr(void);
void app_check_tr_response(int32_t channel);

void PCIE_udmaEventDmaCb(Udma_EventHandle eventHandle, uint32_t eventType,
                              void *appData)
{
    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventType))
    {
            if(eventHandle == &gUdmaCqEventObj[channel_1]) //completed on channel-1
            {
                app_check_tr_response(channel_1);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_2])
            {
                app_check_tr_response(channel_2);
            }
#if defined(NUM_CHANNEL_4) || defined(NUM_CHANNEL_8) || defined(NUM_CHANNEL_16)
            else if(eventHandle == &gUdmaCqEventObj[channel_3])
            {
                app_check_tr_response(channel_3);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_4])
            {
                app_check_tr_response(channel_4);
            }
#endif

#if defined(NUM_CHANNEL_8) || defined(NUM_CHANNEL_16)
            else if(eventHandle == &gUdmaCqEventObj[channel_5])
            {
                app_check_tr_response(channel_5);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_6])
            {
                app_check_tr_response(channel_6);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_7])
            {
                app_check_tr_response(channel_7);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_8])
            {
                app_check_tr_response(channel_8);
            }
#endif

#if defined(NUM_CHANNEL_16)
            else if(eventHandle == &gUdmaCqEventObj[channel_9])
            {
                app_check_tr_response(channel_9);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_10])
            {
                app_check_tr_response(channel_10);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_11])
            {
                app_check_tr_response(channel_11);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_12])
            {
                app_check_tr_response(channel_12);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_13])
            {
                app_check_tr_response(channel_13);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_14])
            {
                app_check_tr_response(channel_14);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_15])
            {
                app_check_tr_response(channel_15);
            }
            else if(eventHandle == &gUdmaCqEventObj[channel_16])
            {
                app_check_tr_response(channel_16);
            }
#endif
    }
            if(
                    (completed_channel_transfer[channel_1] == 1) && (completed_channel_transfer[channel_2] == 1)
#if defined(NUM_CHANNEL_4) || defined(NUM_CHANNEL_8) || defined(NUM_CHANNEL_16)
                    && (completed_channel_transfer[channel_3] == 1)&& (completed_channel_transfer[channel_4] == 1)
#endif
#if defined(NUM_CHANNEL_8) || defined(NUM_CHANNEL_16)
                    && (completed_channel_transfer[channel_5] == 1)&& (completed_channel_transfer[channel_6] == 1)
                    && (completed_channel_transfer[channel_7] == 1)&& (completed_channel_transfer[channel_8] == 1)
#endif
#if defined(NUM_CHANNEL_16)
                    && (completed_channel_transfer[channel_9] == 1)&& (completed_channel_transfer[channel_10] == 1)
                    && (completed_channel_transfer[channel_11] == 1)&& (completed_channel_transfer[channel_12] == 1)
                    && (completed_channel_transfer[channel_13] == 1)&& (completed_channel_transfer[channel_14] == 1)
                    && (completed_channel_transfer[channel_15] == 1)&& (completed_channel_transfer[channel_16] == 1)
#endif
                    )
                SemaphoreP_post(&gBufTransUDMADoneSem);
}

void app_check_tr_response(int32_t channel)
{
    uint64_t        pDesc;
    int32_t status = SystemP_SUCCESS;

    status = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle[channel]), &pDesc);
      if(UDMA_SOK == status)
      {
          /* Check TR response status */
          CacheP_inv(trpdMem[channel], UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
          status = UdmaUtils_getTrpdTr15Response(trpdMem[channel], 1U, 0U);
          if(status != CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE)
          {
          DebugP_log("TR Response failed for transfer : channel = %u\r\n", channel);
          DebugP_assert(FALSE);
          }
          else
          {
          completed_channel_transfer[channel] = 1;
          }
      }

}

int32_t app_udma_event_register(int32_t channel);

int32_t app_udma_event_register(int32_t channel)
{
    int32_t         retVal;
    Udma_EventHandle eventHandle = &gUdmaCqEventObj[channel];

    UdmaEventPrms_init(&eventPrms);
    eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
    eventPrms.chHandle          = gConfigUdma0BlkCopyChHandle[channel];
    eventPrms.eventCb           = &PCIE_udmaEventDmaCb;
    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(drvHandle);

    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
    DebugP_assert(UDMA_SOK == retVal);

return retVal;
}

static void pcieLegacyISR (void *args)
{
    SemaphoreP_post(&gBufTransDoneSem);

    Pcie_rcLegacyIrqEoi (gPcieHandle[CONFIG_PCIE0]);
}

/* Initialize UDMA Trpd */
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

    /* Setup TR */
    pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = length;
    pTr->icnt1    = 1;
    pTr->icnt2    = 1;
    pTr->icnt3    = 1;
    pTr->dim1     = 0;
    pTr->dim2     = 0;
    pTr->dim3     = 0;
    pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
    pTr->dicnt0   = length;
    pTr->dicnt1   = 1;
    pTr->dicnt2   = 1;
    pTr->dicnt3   = 1;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);

    /* Perform cache writeback */
    CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

    return;
}


static void Pcie_udmaBufTransfer_Rx (void)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t timeDiff = 0U;
    uint64_t cycleCount = 0;
    uint32_t length = MIN_PACKET_SIZE;
    int32_t channel = 0;
    uint32_t iter = 0;

    uint64_t cycleCount_T2_T1 = 0;
    uint64_t cycleCount_T1 = 0;
    uint64_t cycleCount_T2 = 0;

    /* Initialize Semaphore */
    status = SemaphoreP_constructBinary(&gBufTransUDMADoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    DebugP_log ("\nStarting PCIe Buffer transfer using UDMA using %d TRPDs in parallel \r\n", NUM_CHANNEL);
    DebugP_log ("cycleCount_T1: Time stamp to submit TRPD \n \r\n");
    DebugP_log ("cycleCount_T2: Time stamp to return descriptor in completion ring \n \r\n");
    DebugP_log ("cycleCount_T3: Time stamp to return legacy ISR cb \n \r\n");
    DebugP_log ("Log \t length \t iteration count \t cycleCount_T1(us) \t cycleCount_T2(us) \t cycleCount_T2_T1(us) \r\n");

    DebugP_log ("\nStarting PCIe Buffer transfer using UDMA \r\n");
    for(channel =0; channel < NUM_CHANNEL; channel++)
    {
        chHandle[channel] = gConfigUdma0BlkCopyChHandle[channel];
        trpdMem[channel] = gUdmaTestTrpdMem[channel];
        trpdMemPhy[channel] = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem[channel], 0U, NULL);

        pcie_BufAddr[channel] = (void *)(CONFIG_PCIE0_OB_REGION0_LOWER + (BUF_SIZE*channel)); //(0x6C000000UL)

        //Enable each channel
        status = Udma_chEnable(chHandle[channel]);
        DebugP_assert(UDMA_SOK == status);

        app_udma_event_register(channel);
        DebugP_assert(UDMA_SOK == status);

    }

    for(length=MIN_PACKET_SIZE; length < MAX_PACKET_SIZE;)
    {
        //Initialize TRPD and channel
        for(channel =0; channel < NUM_CHANNEL; channel++)
        {
            App_udmaTrpdInit(chHandle[channel], trpdMem[channel], &ddr_buf[channel], (void *)(pcie_BufAddr[channel]), length);
            CacheP_wbInv(ddr_buf, length, CacheP_TYPE_ALLD);
            completed_channel_transfer[channel] =0;
        }

        timeDiff = 0;

        cycleCount_T1 = ClockP_getTimeUsec();

        for(channel =0; channel < NUM_CHANNEL; channel++)
        {
            /* Submit TRPD to channel */
           status = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle[channel]), trpdMemPhy[channel]);
           DebugP_assert(UDMA_SOK == status);
        }

        /* Wait for return descriptor in completion ring - this marks transfer completion */
        status = SemaphoreP_pend (&gBufTransUDMADoneSem, SystemP_WAIT_FOREVER);

        if (status != SystemP_SUCCESS)
        {
            DebugP_log("gBufTransUDMADoneSem fail for Time \t %d \t %d \r\n", length, iter);
        }

        cycleCount_T2 = ClockP_getTimeUsec();

        cycleCount_T2_T1 =  cycleCount_T2 - cycleCount_T1;
        DebugP_log("TimeStamp \t %d \t %d \t %llu \t %llu \t %llu \r\n", length, iter, cycleCount_T1, cycleCount_T2, cycleCount_T2_T1);

        length = length + MIN_PACKET_SIZE;
    }

    for(channel =0; channel < NUM_CHANNEL; channel++)
    {
    /* Channel disable */
        status = Udma_chDisable(chHandle[channel], UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        DebugP_assert(UDMA_SOK == status);
    }

    return;
}


void pcie_benchmark_rc_main (void *args)
{

    int32_t status = SystemP_SUCCESS;

    ///Disable sysconfig-UDMA configuration as it is not same what siemens setup have
    Drivers_open();

    Board_driversOpen();

    DebugP_log("Device in RC mode\r\n");

    /* Initialize Semaphore */
    status = SemaphoreP_constructBinary(&gBufTransDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable legacy interrupt and register ISR for legacy IR */
    {
        Pcie_legacyIrqRegisterParam irqParams;

        /* Enable legacy IRQ */
        status =  Pcie_rcLegacyIrqEnable (gPcieHandle[CONFIG_PCIE0], 1);

        if (status == SystemP_SUCCESS)
        {
            /* Register ISR for legacy interrupt */
            irqParams.intNum = 1;
            irqParams.funcPtr = pcieLegacyISR;
            status =  Pcie_rclegacyIrqIsrRegister (gPcieHandle[CONFIG_PCIE0], irqParams);
        }
    }
   /* Pcie buffer transfer using CPU */
    //Pcie_cpuBufTransfer_Rx();
    //Pcie_cpuBufTransfer_Tx();

    /* Pcie buffer transfer using UDMA */
    //Pcie_udmaBufTransfer_Tx();
    Pcie_udmaBufTransfer_Rx();
    //ddr_udmaBufTransfer();

    DebugP_log("\nPCIE benchmark example completed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();

    return;
}
