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

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{
    Drivers_ospiOpen();
    Drivers_udmaOpen();
    Drivers_uartOpen();
}

void Drivers_close(void)
{
    Drivers_ospiClose();
    Drivers_udmaClose();
    Drivers_uartClose();
}

/*
 * OSPI
 */
/* OSPI Driver handles */
OSPI_Handle gOspiHandle[CONFIG_OSPI_NUM_INSTANCES];

/* OSPI Driver Parameters */
OSPI_Params gOspiParams[CONFIG_OSPI_NUM_INSTANCES] =
{
    {
        .ospiDmaChIndex = 0,
    },
};

void Drivers_ospiOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_OSPI_NUM_INSTANCES; instCnt++)
    {
        gOspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_OSPI_NUM_INSTANCES; instCnt++)
    {
        gOspiHandle[instCnt] = OSPI_open(instCnt, &gOspiParams[instCnt]);
        if(NULL == gOspiHandle[instCnt])
        {
            DebugP_logError("OSPI open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_ospiClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_ospiClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_OSPI_NUM_INSTANCES; instCnt++)
    {
        if(gOspiHandle[instCnt] != NULL)
        {
            OSPI_close(gOspiHandle[instCnt]);
            gOspiHandle[instCnt] = NULL;
        }
    }

    return;
}

/*
 * UDMA
 */

void Drivers_udmaOpen(void)
{
}

void Drivers_udmaClose(void)
{
}


/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

#include <drivers/uart/v0/lld/dma/uart_dma.h>
#include <kernel/dpl/ClockP.h>

UART_DmaChConfig gUartDmaChConfig[CONFIG_UART_NUM_INSTANCES] =
{
                NULL,
};

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
        {
            .baudRate           = 115200,
            .dataLength         = UART_LEN_8,
            .stopBits           = UART_STOPBITS_1,
            .parityType         = UART_PARITY_NONE,
            .readMode           = UART_TRANSFER_MODE_BLOCKING,
            .readReturnMode     = UART_READ_RETURN_MODE_FULL,
            .writeMode          = UART_TRANSFER_MODE_BLOCKING,
            .readCallbackFxn    = NULL,
            .writeCallbackFxn   = NULL,
            .hwFlowControl      = FALSE,
            .hwFlowControlThr   = UART_RXTRIGLVL_16,
            .transferMode       = UART_CONFIG_MODE_POLLED,
            .skipIntrReg        = FALSE,
            .uartDmaIndex = -1,
            .intrNum            = 210U,
            .intrPriority       = 4U,
            .operMode           = UART_OPER_MODE_16X,
            .rxTrigLvl          = UART_RXTRIGLVL_8,
            .txTrigLvl          = UART_TXTRIGLVL_32,
            .rxEvtNum           = 0U,
            .txEvtNum           = 0U,
        },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}

