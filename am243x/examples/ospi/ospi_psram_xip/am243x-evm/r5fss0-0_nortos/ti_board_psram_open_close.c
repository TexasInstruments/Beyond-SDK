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
#include "ti_board_psram_open_close.h"

#include <kernel/dpl/DebugP.h>

/*
 * PSRAM
 */
/* PSRAM Object - initialized during Psram_open() */
Psram_OspiObject gPsramOspiObject_APS6408LOBMXBA;

/* PSRAM Driver handles - opened during Board_psramOpen() */
Psram_Handle gPsramHandle[CONFIG_PSRAM_NUM_INSTANCES];

/* PSRAM Attrs */
Psram_Attrs gPsramAttrs_APS6408LOBMXBA =
{
    .psramName = "APS6408LOBMXBA",
    .psramSize = 8388608,
};

/* PSRAM Config */
Psram_Config gPsramConfig[CONFIG_PSRAM_NUM_INSTANCES] =
{
{
    .attrs = &gPsramAttrs_APS6408LOBMXBA,
    .fxns = &gPsramOspiFxns,
    //.devConfig = &gPsramOspiDevCfg_APS6408LOBMXBA,
    .object = (void *)&gPsramOspiObject_APS6408LOBMXBA,
},
};
//PC-- ToDo is below needed?
/* PSRAM Open Params - populated from SysConfig options */
Psram_Params gPsramParams[CONFIG_PSRAM_NUM_INSTANCES] =
{
    {
        .quirksFxn = NULL,
        .custProtoFxn = NULL,
    },
};

uint32_t gPsramConfigNum = CONFIG_PSRAM_NUM_INSTANCES;

int32_t Board_psramOpen()
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_PSRAM_NUM_INSTANCES; instCnt++)
    {
        gPsramHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Set the underlying driver instance to the PSRAM config */
    gPsramAttrs_APS6408LOBMXBA.driverInstance = CONFIG_OSPI0;

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_PSRAM_NUM_INSTANCES; instCnt++)
    {

        gPsramHandle[instCnt] = Psram_open(instCnt, &gPsramParams[instCnt]);
        if(NULL == gPsramHandle[instCnt])
        {
            DebugP_logError("PSRAM open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Board_psramClose();   /* Exit gracefully */
    }
    return status;
}

void Board_psramClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_PSRAM_NUM_INSTANCES; instCnt++)
    {
        if(gPsramHandle[instCnt] != NULL)
        {
            Psram_close(gPsramHandle[instCnt]);
            gPsramHandle[instCnt] = NULL;
        }
    }
    return;
}
