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

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/sciclient.h>
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


/* 
 * GPIO INTRTR bank interrupt source index base 
 * See https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#mcu-mcu-gpiomux-introuter0-interrupt-router-input-sources
 */
#define TISCI_GPIO_BANK_INT_SRC_IDX_BASE_GPIO0      ( 90U )
#define TISCI_GPIO_BANK_INT_SRC_IDX_BASE_GPIO1      ( 90U )
#define TISCI_GPIO_BANK_INT_SRC_IDX_BASE_MCU_GPIO0  ( 30U )
#define TISCI_GPIO1_BANK_INT_SRC_IDX(x)             ( TISCI_GPIO_BANK_INT_SRC_IDX_BASE_GPIO1 + x )

/* 
 * GPIO INTRTR destination index
 * See https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#main-gpiomux-introuter0-interrupt-router-output-destinations
 */
#define L2G_EVENT_ID0                               ( 24U )

/* Debug: route GPIO interrupt to VIM/R5F, confirm interrupt input is working properly */
//#define _DBG_GPIO_INT_ENABLE
#ifdef _DBG_GPIO_INT_ENABLE
#define R5F_VIM_INT_ID0                             ( 40 )
static void GPIO_bankIsrFxn(void *args);
HwiP_Object gGpioBankHwiObject;
volatile uint32_t gGpioIntrDone = 0;
#endif

uint32_t gGpioBaseAddr = CONFIG_GPIO1_BASE_ADDR;

int32_t gpioIrInit(
    uint16_t src_id,
    uint16_t src_index,
    uint16_t dst_id,
    uint16_t dst_host_irq);

int32_t gpioIrDeInit(
    uint16_t src_id,
    uint16_t src_index,
    uint16_t dst_id,
    uint16_t dst_host_irq);

/* Number of bytes to do memcpy */
#define UDMA_TEST_NUM_BYTES             (1024U)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Application Buffers */
uint8_t gUdmaTestSrcBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTestDestBuf[UDMA_ALIGN_SIZE(UDMA_TEST_NUM_BYTES)] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);
static void App_udmaInitBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);
static void App_udmaCompareBuf(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);

static void configure_intaggrL2G(uint32_t localEvent, uint32_t globalEvent);

uint32_t gXferErrCnt = 0U;

volatile uint8_t gRunFlag = 1;


void *gpio_trigger_bcdma_main(void *args)
{
    int32_t         retVal = UDMA_SOK;
    Udma_ChHandle   chHandle;
    uint8_t        *srcBuf = &gUdmaTestSrcBuf[0U];
    uint8_t        *destBuf = &gUdmaTestDestBuf[0U];
    uint32_t        length = UDMA_TEST_NUM_BYTES;
    uint8_t        *trpdMem = &gUdmaTestTrpdMem[0U];
    uint64_t        trpdMemPhy;
    uint32_t        globalEvent0;
    uint32_t        gpioBankNum;
    uint32_t        gpioIntrStatus;
#ifdef _DBG_GPIO_INT_ENABLE
    HwiP_Params     hwiPrms;
#endif
    uint32_t        count = 0;
    
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Initialize MAIN domain GPIO IR routes */
#ifndef _DBG_GPIO_INT_ENABLE
    /* Configure GPIO interrupt route to interrupt aggregator */
    retVal = gpioIrInit(TISCI_DEV_GPIO1,
        TISCI_GPIO1_BANK_INT_SRC_IDX(2),
        TISCI_DEV_DMASS0_INTAGGR_0,
        L2G_EVENT_ID0);
    if (retVal != 0)
    {
        DebugP_assert(FALSE);
    }
#else
    /* Debug: configure GPIO interrupt route to R5F core
       NOTE: SYSFW enforces a single destination for each interrupt through the router.
             Hence, GPIO interrupt can't be simultaneously routed to Interrupt Aggregator and R5F core.
     */
    retVal = gpioIrInit(TISCI_DEV_GPIO1,
        TISCI_GPIO1_BANK_INT_SRC_IDX(3),
        TISCI_DEV_R5FSS0_CORE0,
        R5F_VIM_INT_ID0,
        TISCI_HOST_ID_MAIN_0_R5_1);
    if (retVal != 0)
    {
        DebugP_assert(FALSE);
    }
#endif    

    DebugP_log("GPIO Trigger BCDMA Test Started ...\r\n");

    /* 
     *  Configure GPIO
     */

    /* Address translate for GPIO base address */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);
    /* Setup GPIO for interrupt generation */
    GPIO_setDirMode(gGpioBaseAddr, CONFIG_GPIO1_PIN, CONFIG_GPIO1_DIR);
    GPIO_setTrigType(gGpioBaseAddr, CONFIG_GPIO1_PIN, CONFIG_GPIO1_TRIG_TYPE);

#ifdef _DBG_GPIO_INT_ENABLE
    /* Register GPIO Bank interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = R5F_VIM_INT_ID0;
    hwiPrms.callback = &GPIO_bankIsrFxn;
    hwiPrms.args     = (void *) CONFIG_GPIO0_PIN;
    retVal = HwiP_construct(&gGpioBankHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );
#endif

    /* Enable GPIO Bank interrupts */
    gpioBankNum = GPIO_GET_BANK_INDEX(CONFIG_GPIO1_PIN);
    GPIO_bankIntrEnable(gGpioBaseAddr, gpioBankNum);


    /* 
     *  Configure UDMA / BCDMA 
     */

    /* Get UDMA handle */
    chHandle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */

    /* Init buffers and TR packet descriptor */
    App_udmaInitBuf(srcBuf, destBuf, length);
    App_udmaTrpdInit(chHandle, trpdMem, destBuf, srcBuf, length);

    /* Get physical memory address for TRPD */
    trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);
    
    /* Submit TRPD to channel */
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);

    /* Channel enable */
    retVal = Udma_chEnable(chHandle);
    DebugP_assert(UDMA_SOK == retVal);

    /* Get UDMA event trigger */
    globalEvent0 = Udma_chGetTriggerEvent(chHandle, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);

    /* Configure Interrupt Aggregator, L2G */
    configure_intaggrL2G(L2G_EVENT_ID0, globalEvent0);

    while (gRunFlag == 1)
    {
        do {
            gpioIntrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, gpioBankNum);
        } while ((gpioIntrStatus == 0) && (gRunFlag == 1));
        GPIO_clearBankIntrStatus(gGpioBaseAddr, gpioBankNum, gpioIntrStatus);

        /* Compare data */
        App_udmaCompareBuf(srcBuf, destBuf, length);
        
        count++;
        DebugP_log("Data Transfer!!: %d\r\n", count);
    }

    /* Channel disable */
    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    DebugP_assert(UDMA_SOK == retVal);

    /* De-initialize IR routes for this R5F core */
    /* Board_gpioDeinit(); */
    retVal = gpioIrDeInit(TISCI_DEV_GPIO1, 
        TISCI_GPIO1_BANK_INT_SRC_IDX(3), 
        TISCI_DEV_DMASS0_INTAGGR_0, 
        L2G_EVENT_ID0);
    if (retVal != 0)
    {
        DebugP_assert(FALSE);
    }

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

    /* Set TR descriptor reload information */
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
    pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
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
    CacheP_wb(destBuf, length, CacheP_TYPE_ALLD); 

    if (flag)
    {
        gXferErrCnt++;
        DebugP_log("Error count %d: gXferErrCnt");
    }

    return;
}

static void configure_intaggrL2G(uint32_t localEvent, uint32_t globalEvent)
{
    uint64_t eventRegOffset = CSL_DMASS0_INTAGGR_L2G_BASE + (localEvent * 0x20U);
    CSL_REG32_WR(eventRegOffset, ( (0U << 31U) | (globalEvent & 0xFFFFU) ) ); /* pulse event */
}

#ifdef _DBG_GPIO_INT_ENABLE
static void GPIO_bankIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args;
    uint32_t    bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t    intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);

    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    /* Per pin interrupt handling */
    if(intrStatus & pinMask)
    {
        gGpioIntrDone++;
    }
}
#endif


int32_t gpioIrInit(
    uint16_t src_id,
    uint16_t src_index,
    uint16_t dst_id,
    uint16_t dst_host_irq)
{
    int32_t                             retVal;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;


    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = src_id;
    rmIrqReq.dst_id                 = dst_id;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    rmIrqReq.src_index              = src_index;
    rmIrqReq.dst_host_irq           = dst_host_irq;
    
    retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    if (retVal != 0)
    {
        DebugP_log("[Error] Sciclient event config failed!!!\r\n");
    }
    
    return retVal;
}

int32_t gpioIrDeInit(
    uint16_t src_id,
    uint16_t src_index,
    uint16_t dst_id,
    uint16_t dst_host_irq)
{
    int32_t                             retVal;
    struct tisci_msg_rm_irq_release_req rmIrqReq;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = src_id;
    rmIrqReq.dst_id                 = dst_id;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    rmIrqReq.src_index              = src_index;
    rmIrqReq.dst_host_irq           = dst_host_irq;

    retVal = Sciclient_rmIrqRelease(&rmIrqReq, SystemP_WAIT_FOREVER);
    if (retVal != 0)
    {
        DebugP_log("[Error] Sciclient event reset failed!!!\r\n");
    }
    
    return retVal;
}


int32_t gpioIrInitRaw(
    uint16_t src_index,
    uint16_t dst_index
)
{
    int32_t                             retVal;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0;
    rmIrqReq.src_index              = src_index;
    rmIrqReq.dst_id                 = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0;
    rmIrqReq.dst_host_irq           = dst_index;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    retVal = Sciclient_rmIrqSetRaw(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    if (retVal != 0)
    {
        DebugP_log("[Error] Sciclient event config failed!!!\r\n");
    }

    return retVal;
}

