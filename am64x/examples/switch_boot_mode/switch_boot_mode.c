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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/soc/am64x_am243x/soc.h>
#include <stdio.h>

/*
 * This is an switch boot mode project provided for r5fss0-0 cores present in the device.
 * User can use this project to change boot mode without disturbing the boot pins.
 *
 * To run the project 
 * Set the board in NO BOOT mode.
 * Launch the target configuartion file and open scripting console.
 * In the scripting console run below command  
 *     -> loadJSFile "<SDK Install path>/tools/ccs_load/am64x/load_dmsc.js"
 * Load the .out file on the r5fss0-0 core.
 */

#define UART 0xD3B  // UART BOOT MODE CONFIG
#define OSPI 0x273  // OSPI BOOT MODE CONFIG
#define SD   0x36C3 // SD BOOT MODE CONFIG
#define NO 0xFB     // NO BOOT MODE CONFIG
#define EMMC 0x4B   // EMMC BOOT MODE CONFIG


void switch_boot_mode_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    printf("Press the corresponding number to switch boot mode\r\n\r\n");
    printf("1. UART BOOT MODE\r\n");
    printf("2. OSPI BOOT MODE\r\n");
    printf("3. SD BOOT MODE\r\n");
    printf("4. NO BOOT MODE\r\n");
    printf("5. EMMC BOOT MODE\r\n\r\n");

    int mode;
    printf("Enter Boot Mode : ");
    scanf("%d", &mode);
    printf("\r\n");

    // Setting the Board in specified boot mode
    switch(mode)
    {
        case 1:
            SOC_setDevStat(UART);
            printf("Switched UART BOOT MODE\r\n");
            break;
        case 2:
            SOC_setDevStat(OSPI);
            printf("Switched OSPI BOOT MODE\r\n");
            break;
        case 3:
            SOC_setDevStat(SD);
            printf("Switched SD BOOT MODE\r\n");
            break;
        case 4:
            SOC_setDevStat(NO);
            printf("Switched NO BOOT MODE\r\n");
            break;
        case 5:
            SOC_setDevStat(EMMC);
            printf("Switched EMMC BOOT MODE\r\n");
            break;
        default:
            printf("INVALID BOOT MODE\r\n");
            break;
    }

    SOC_generateSwWarmResetMainDomain(); // Performing Software warm reset of MAIN Domain

    /* Close drivers to close the UART driver for console */
    Board_driversClose();
    Drivers_close();
}
