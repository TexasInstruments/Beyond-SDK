/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <board/psram.h>
#include "ti_drivers_open_close.h"
#include "psram_ospi.h"
#include "ti_board_open_close.h"
#include "ti_board_psram_open_close.h"

#define APP_OSPI_PSRAM_OFFSET_BASE  (0x00U)

#define APP_OSPI_DATA_SIZE (1024)
uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE];
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gOspiRxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(128U)));

void ospi_psram_io_fill_buffers(void);
int32_t ospi_psram_io_compare_buffers(void);
extern Psram_Config gPsramConfig[];

void ospi_psram_basic_init_main(void *args)
{
    int32_t status = SystemP_SUCCESS;

    /* Open OSPI Driver, among others */
    Drivers_open();
	i2c_io_expander(NULL);

    /* Open Psram drivers with OSPI instance as input */
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);
}

void BoardDiag_OspiTest(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    Psram_OspiObject *obj = NULL;
    Psram_Config *config;

    /* Open OSPI Driver, among others */
    Drivers_open();
    status = i2c_io_expander(NULL);
    /* Open Psram drivers with OSPI instance as input */
    status = Board_driversOpen();
    status = Board_psramOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */
    config = &gPsramConfig[CONFIG_PSRAM0];
    obj = (Psram_OspiObject*)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    obj->syncModeEnable = 0x0;
    
    /* This application writes and reads from the RAM both in INDAC and DAC mode */
    /* Write=DAC and Read=INDAC is not supported */
    offset = APP_OSPI_PSRAM_OFFSET_BASE;
    ospi_psram_io_fill_buffers();
    obj->dacEnable = TRUE;
    Psram_write(gPsramHandle[CONFIG_PSRAM0], offset, gOspiTxBuf, APP_OSPI_DATA_SIZE);
    obj->dacEnable = TRUE;
    Psram_read(gPsramHandle[CONFIG_PSRAM0], offset, gOspiRxBuf, APP_OSPI_DATA_SIZE);
    status |= ospi_psram_io_compare_buffers();

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Board_psramClose();
    Drivers_close();
}

void ospi_psram_io_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        gOspiTxBuf[i] = i;
        gOspiRxBuf[i] = 0U;
    }
}

int32_t ospi_psram_io_compare_buffers(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        if(gOspiTxBuf[i] != gOspiRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("OSPI read data mismatch @ i= %ld!!!\r\n",i);
            break;
        }
    }
    return status;
}
