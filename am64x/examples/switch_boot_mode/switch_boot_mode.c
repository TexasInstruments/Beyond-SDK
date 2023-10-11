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

/*
 * This is an switch_boot_mode project provided for R5F0-0 core present in the device.
 * User can use this project to switch between various boot modes available on device.
 *
 * This application does driver and board init and implements a interface for user to select between various boot mode settings.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */

#define UART    0xD3B
#define OSPI    0x273
#define SDBOOT  0x36C3
#define EMMC    0x4B
#define DFU     0x53
#define DEV     0x7B
#define PCIE    0x6B


void switch_boot_mode_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    printf("Press the corresponding number to switch boot mode\r\n\r\n");
    printf("1. UART BOOT MODE\r\n");
    printf("2. OSPI BOOT MODE\r\n");
    printf("3. SD BOOT MODE\r\n");
    printf("4. EMMC BOOT MODE\r\n");
    printf("5. DFU BOOT MODE\r\n");
    printf("6. DEV BOOT MODE\r\n");
    printf("7. PCIE BOOT MODE\r\n\r\n");

    int mode;
    printf("Enter Boot Mode : ");
    scanf("%d", &mode);
    printf("\r\n");
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
            SOC_setDevStat(SDBOOT);
            printf("Switched SD BOOT MODE\r\n");
            break;
        case 4:
            SOC_setDevStat(EMMC);
            printf("Switched EMMC BOOT MODE\r\n");
            break;
        case 5:
            SOC_setDevStat(DFU);
            printf("Switched DFU BOOT MODE\r\n");
            break;
        case 6:
            SOC_setDevStat(DEV);
            printf("Switched DEV BOOT MODE\r\n");
            break;
        case 7:
            SOC_setDevStat(PCIE);
            printf("Switched PCIE BOOT MODE\r\n");
            break;
        default:
            printf("INVALID BOOT MODE\r\n");
            break;
    }

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 0);
    SOC_generateSwWarmResetMainDomain();

    Board_driversClose();
    Drivers_close();
}
