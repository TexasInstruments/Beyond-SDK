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


/**
 * @file EMMC_API.c
 * @brief EMMC Memory API implementation for TI AM2432 Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements EMMC communication APIs including read/write operations.
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_MMCSD_START_BLK (0x300000U) /* @1.5GB */
#define APP_MMCSD_MAX_SIZE  (0x1AA4000U) /* Total Number of writable blocks (Each block is 512 bytes)*/

int32_t EMMC_ReadWrite(uint32_t offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData)
{
    int32_t status = SystemP_SUCCESS;

    uint32_t blockSize = MMCSD_getBlockSize(gMmcsdHandle[CONFIG_MMCSD0]);
    uint32_t numBlocks = nBytes / blockSize;
    uint8_t tmpData[1024] = {0};

    if((nBytes % blockSize) != 0)
    {
        numBlocks += 1;
    }

    /* Starting 3MB partition is reserved for boot partition and should not be used */
    uint32_t startAddress = offset + APP_MMCSD_START_BLK;
    uint32_t endAddress = APP_MMCSD_START_BLK + APP_MMCSD_MAX_SIZE;
    if(startAddress >= endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Offset or nBytes value exceeding the available memory range.\r\n");
        return status;
    }
    if(nBytes > 128 || nBytes <= 0)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Data size should be in range 1 to 128 bytes.\r\n");
        return status;
    }

    /* Store data from the EMMC as we have to do read modified block write */
    status = MMCSD_read(gMmcsdHandle[CONFIG_MMCSD0], tmpData, startAddress, numBlocks);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("Not able to access EMMC Memory.\r\n");
        return status;
    }

    /* copy the value of writeData to tmpData */
    for(int i=0; i<nBytes; i++)
    {
        tmpData[i] = writeData[i];
    }

    /* Write data to EMMC */
    status = MMCSD_write(gMmcsdHandle[CONFIG_MMCSD0], tmpData, startAddress, numBlocks);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("EMMC write failed.\r\n");
        return status;
    }

    /* Read back written data */
    status = MMCSD_read(gMmcsdHandle[CONFIG_MMCSD0], readData, startAddress, numBlocks);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("EMMC read failed.\r\n");
        return status;
    }

    DebugP_logInfo("EMMC read/write test have passed.\r\n");
    return status;
}
