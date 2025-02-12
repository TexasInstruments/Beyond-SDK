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

#define APP_OSPI_DATA_SIZE (32)
uint8_t gPsramBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(32), section(".psram_buf")));
__attribute__ ((section(".psram_code"))) void testcode(void);

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
    uint32_t offset = 0x0;
    Psram_OspiObject *obj = NULL;
    Psram_Config *config;

    /* Open OSPI Driver, among others */
    Drivers_open();
    status = i2c_io_expander(NULL);
    /* Open Psram drivers with OSPI instance as input */
    status = Board_driversOpen();
    status = Board_psramOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    /******************************* XIP *******************************/

    config = &gPsramConfig[CONFIG_PSRAM0];
    obj = (Psram_OspiObject*)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    obj->syncModeEnable = 0x0;
    const OSPI_Attrs *attrs = ((OSPI_Config *)obj->ospiHandle)->attrs;
    OSPI_Object *obj_ospi = ((OSPI_Config *)obj->ospiHandle)->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
    obj->dacEnable = TRUE;
    obj_ospi->isDacEnable = TRUE;

    uint32_t srcAddr = (uint32_t)0x70140000; //MSRAM address where we manually put testcode() in our hex file
    uint32_t dstAddr = (uint32_t)0x70150000; //MSRAM address where we will copy back from PSRAM to check data integrity
    uint32_t nBytes = 0x200;

    /******************************* COPY APPLICATION *******************************/
    //Copy testcode() from MSMRAM to PSRAM
    Psram_write(gPsramHandle[CONFIG_PSRAM0], offset, (uint8_t *)srcAddr, nBytes);
    //Copy testcode() from PSRAM to MSMRAM to check data integrity
    Psram_read(gPsramHandle[CONFIG_PSRAM0], offset, (uint8_t *)dstAddr, nBytes);

    /******************************* DATA CHECK *******************************/

    uint32_t error = 0;
    uint32_t i, data1, data2;
    for (i = 0 ; i < (nBytes) ; i=i+4)
    {
       data1 = *((volatile uint32_t *)(srcAddr + i));
       data2 = *((volatile uint32_t *)(dstAddr + i));
       if (data1 != data2)
       {
           DebugP_log("Data at source location 0x%x = 0x%x and ", (srcAddr + i), data1);
           DebugP_log("Data at destination location 0x%x = 0x%x \n", (dstAddr + i), data2);
           error = 1;
       }
    }

    if (error == 0)
    {
        DebugP_log("PSRAM code write pass\n");
    }
    else
    {
        DebugP_log("PSRAM code write fail\n");
    }

    // Re-write  MSMRAM with 0x0 to ensure we will run testcode() from PSRAM
    memset((void *)0x70140000, 0, nBytes);
    memset((void *)0x70150000, 0, nBytes);


    /******************************* XIP TESTCASE *******************************/
    DebugP_log("Starting XIP\n");

    testcode();

    Board_driversClose();
    Board_psramClose();
    Drivers_close();
}

__attribute__ ((section(".psram_code"))) void testcode(void)
{

    uint32_t pattern3 = 0xDDDDDDDD;
    memset(gPsramBuf,pattern3,4);
    DebugP_log("Hi, I am executing from PSRAM!\n");

    uint32_t a =1;
    uint32_t b =2;
    uint32_t c;
    c=a+2;

/******************************* Direct memory manipulation *******************************/

uint32_t* wPtr = (uint32_t*)0x70080000;
uint32_t* rPtr = (uint32_t*)0x70090000;

// Constant value to be written to memory
const uint32_t val = 0x1234ABCD;

// Write the value to four consecutive memory locations starting at 0x70080000
*wPtr++ = val;
*wPtr++ = val;
*wPtr++ = val;
*wPtr = val;

// Reset wPtr to the original address 0x70080000
wPtr = (uint32_t*)0x70080000;

// Copy the values from the memory range starting at 0x70080000 to the memory range starting at 0x70090000
*rPtr++ = *wPtr++;
*rPtr++ = *wPtr++;
*rPtr++ = *wPtr++;
*rPtr = *wPtr;

DebugP_log("Finished OSPI PSRAM XIP test!\n");
}
