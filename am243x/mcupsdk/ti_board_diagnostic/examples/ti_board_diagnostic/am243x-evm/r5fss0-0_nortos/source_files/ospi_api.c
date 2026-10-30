/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 * @file OSPI_API.c
 * @brief OSPI FLASH API implementation for TI AM2432 Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements OSPI Flash communication APIs including read, write operations.
 */
 
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define APP_OSPI_FLASH_OFFSET_BASE  (0x200000U)
#define APP_OSPI_FLASH_END_ADDRESS  (0x3F00000U) /* @63 MB */

int32_t OSPI_ReadWrite(uint32_t offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t blk, page;

    /* Offset values should always start with 0x200000, because of the early region occupied by bootloader. */
    offset += APP_OSPI_FLASH_OFFSET_BASE; 
    uint32_t endAddress = APP_OSPI_FLASH_END_ADDRESS;
    /* The offset should be 256KB align */
    if(offset % (256*1024)) 
    {
        status = SystemP_FAILURE;
        DebugP_logError("Offset is not 256KB align.\r\n");
        return status;
    }
    if(offset >= endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Offset is out of available OSPI FLASH range.\r\n");
        return status;
    }
    if(nBytes > 128 || nBytes <= 0)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Data size should be in range 1 to 128 bytes.\r\n");
        return status;
    }
    Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);

    status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
    if(SystemP_SUCCESS != status)
    {
        DebugP_logError("Block Erase Failed at 0x%X offset !!!\r\n", offset);
    }

    if(SystemP_SUCCESS == status)
    {
        /** Marking the end of write buffer.
         * required to make the write buffer length even,
         * because write for odd length fails.
        */
        writeData[nBytes] = 0xFF;
        uint32_t bytesToWrite = (nBytes%2)? (nBytes+1): (nBytes);

        status = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, writeData, bytesToWrite);
        if(SystemP_SUCCESS != status)
        {
            DebugP_logInfo("Flash Write of %d bytes failed at 0x%X offset !!!\r\n", nBytes, offset);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        status = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, readData, nBytes);
        if(SystemP_SUCCESS != status)
        {
            DebugP_logError("Flash Read of %d bytes failed at 0x%X offset !!!\r\n", nBytes, offset);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_logInfo("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_logError("Some tests have failed!!\r\n");
    }
    return status;
}
