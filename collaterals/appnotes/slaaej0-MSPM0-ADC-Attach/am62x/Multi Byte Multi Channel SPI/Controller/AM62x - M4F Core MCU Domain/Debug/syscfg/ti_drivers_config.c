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
/*
 * Auto generated file
 */

#include "ti_drivers_config.h"
#include <drivers/sciclient.h>

/*
 * MCSPI
 */

/* MCSPI atrributes */
static MCSPI_Attrs gMcspiAttrs[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_MCU_MCSPI0_CFG_BASE,
        .inputClkFreq       = 50000000U,
        .intrNum            = 38,
        .operMode           = MCSPI_OPER_MODE_INTERRUPT,
        .intrPriority       = 4U,
        .chMode             = MCSPI_CH_MODE_SINGLE,
        .pinMode            = MCSPI_PINMODE_4PIN,
        .initDelay          = MCSPI_INITDLY_0,
    },
};
/* MCSPI objects - initialized by the driver */
static MCSPI_Object gMcspiObjects[CONFIG_MCSPI_NUM_INSTANCES];
/* MCSPI driver configuration */
MCSPI_Config gMcspiConfig[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        &gMcspiAttrs[CONFIG_MCSPI0],
        &gMcspiObjects[CONFIG_MCSPI0],
    },
};

uint32_t gMcspiConfigNum = CONFIG_MCSPI_NUM_INSTANCES;

#include <drivers/mcspi/v0/dma/mcspi_dma.h>
MCSPI_DmaConfig gMcspiDmaConfig =
{
    .fxns        = NULL,
    .mcspiDmaArgs = (void *)NULL,
};


uint32_t gMcspiDmaConfigNum = CONFIG_MCSPI_NUM_DMA_INSTANCES;

/*
 * UART
 */

/* UART atrributes */
static UART_Attrs gUartAttrs[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_MCU_UART0_BASE,
        .inputClkFreq       = 48000000U,
    },
};
/* UART objects - initialized by the driver */
static UART_Object gUartObjects[CONFIG_UART_NUM_INSTANCES];
/* UART driver configuration */
UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES] =
{
    {
        &gUartAttrs[CONFIG_UART0],
        &gUartObjects[CONFIG_UART0],
    },
};

uint32_t gUartConfigNum = CONFIG_UART_NUM_INSTANCES;

#include <drivers/uart/v0/dma/uart_dma.h>


UART_DmaConfig gUartDmaConfig[CONFIG_UART_NUM_DMA_INSTANCES] =
{
};

uint32_t gUartDmaConfigNum = CONFIG_UART_NUM_DMA_INSTANCES;


void Drivers_uartInit(void)
{
    UART_init();
}


void Pinmux_init();
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */
void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();
    /* We should do sciclient init before we enable power and clock to the peripherals */
    /* SCICLIENT init */
    {

        int32_t retVal = SystemP_SUCCESS;

        retVal = Sciclient_init(CSL_CORE_ID_M4FSS0_0);
        DebugP_assertNoLog(SystemP_SUCCESS == retVal);

    }
    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    MCSPI_init();
    Drivers_uartInit();
}

void System_deinit(void)
{
    MCSPI_deinit();
    UART_deinit();
    PowerClock_deinit();
    /* SCICLIENT deinit */
    {
        int32_t         retVal = SystemP_SUCCESS;

        retVal = Sciclient_deinit();
        DebugP_assertNoLog(SystemP_SUCCESS == retVal);
    }
    Dpl_deinit();
}
