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
#include "ti_board_open_close.h"
#include "ti_board_psram_open_close.h"

#define APP_OSPI_PSRAM_OFFSET_BASE  (0x200000U)
#define ITERATIONS (10U)
#define APP_OSPI_DATA_SIZE (512*1024)
uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE];
/* read buffer MUST be cache line aligned when using DMA, we align to 128B though 32B is enough */
uint8_t gOspiRxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(1024U)));

void ospi_psram_io_fill_buffers(void);
int32_t ospi_psram_io_compare_buffers(void);
extern Psram_Config gPsramConfig[];

void ospi_psram_io_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    uint64_t startTime = 0U, endTime = 0U;
    float writeDuration = 0U, readDuration = 0U;

    /* Write & read speed in MiBps (Mebibytes per second)*/
    float writeSpeed = 0;
    float readSpeed = 0;

    Drivers_open();
    status = i2c_io_expander(NULL);
    status = Board_psramOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    for(uint32_t i = 0; i < ITERATIONS; i++)
    {
        offset = APP_OSPI_PSRAM_OFFSET_BASE;
        ospi_psram_io_fill_buffers();
        
        startTime = ClockP_getTimeUsec();
        status = Psram_write(gPsramHandle[CONFIG_PSRAM0], offset, gOspiTxBuf, APP_OSPI_DATA_SIZE);
        endTime = ClockP_getTimeUsec();
        DebugP_assert(status == SystemP_SUCCESS);
        
        writeDuration = writeDuration + (endTime - startTime);

        startTime = ClockP_getTimeUsec();
        status = Psram_read(gPsramHandle[CONFIG_PSRAM0], offset, gOspiRxBuf, APP_OSPI_DATA_SIZE);
        endTime = ClockP_getTimeUsec();
        DebugP_assert(status == SystemP_SUCCESS);
        
        readDuration = readDuration + (endTime - startTime);

        status = ospi_psram_io_compare_buffers();
        DebugP_assert(status == SystemP_SUCCESS);

        DebugP_log("All Tests Passed !!!!\r\n");
    }
    
    writeDuration = writeDuration / ITERATIONS;
    readDuration = readDuration / ITERATIONS;

    DebugP_log("Write duration: %u us\r\n",(uint32_t) writeDuration); 
    DebugP_log("Read duration: %u us\r\n",(uint32_t) readDuration);

    writeSpeed = ((float)APP_OSPI_DATA_SIZE)/(writeDuration);
    readSpeed = ((float)APP_OSPI_DATA_SIZE)/(readDuration);

    DebugP_log("Write Speed: %f MiBps\r\n", writeSpeed); 
    DebugP_log("Read Speed: %f MiBps\r\n", readSpeed);

    Board_driversClose();
    Board_psramClose();
    i2c_io_expander_set2flash(NULL);
    Drivers_close();
}

void ospi_psram_io_fill_buffers(void)
{
    uint32_t i;
    uint32_t seed = CycleCounterP_getCount32();
    uint32_t val = seed + 12345U;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        /* generate random numbers */
        val = val * 1103515245U + i;  
        gOspiTxBuf[i] = (val ^ (val >> 8)) & 0xFF;
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
            DebugP_logError("OSPI read data mismatch @ i=%ld!!!\r\n",i);
            break;
        }
    }
    return status;
}
