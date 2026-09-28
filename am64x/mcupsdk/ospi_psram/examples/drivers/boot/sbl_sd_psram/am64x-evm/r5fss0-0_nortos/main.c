/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_elf.h>
#include <board/psram.h>
#include "ti_board_psram_open_close.h"
#include <drivers/bootloader/bootloader_priv.h>
#include <kernel/dpl/CacheP.h>

#define BOOTLOADER_SD_APPIMAGE_FILENAME ("/sd0/app")
#define BOOTLOADER_SD_APPIMAGE_XIP_FILENAME ("/sd0/app_xip")

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x90000) /* 576 KB for regular app */
#define XIP_STREAMING_CHUNK_SIZE (256 * 1024) /* 256 KB streaming chunks */

uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
uint8_t gXipStreamBuf[XIP_STREAMING_CHUNK_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

extern Psram_Config gPsramConfig[];

/* call this API to stop the booting process and spin, so that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

int32_t Bootloader_programXipImageStreaming(FF_FILE *appXipFp, uint32_t xipImageSize)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t elfHeader[256];  /* Small buffer for ELF header */

    /* Read ELF header from file */
    ff_fseek(appXipFp, 0, FF_SEEK_SET);
    size_t bytesRead = ff_fread(elfHeader, 1, sizeof(elfHeader), appXipFp);
    if (bytesRead < sizeof(Bootloader_ELFH32))
    {
        DebugP_log("ERROR: Failed to read ELF header\r\n");
        return SystemP_FAILURE;
    }

    /* Parse ELF header */
    Bootloader_ELFH32 *elfHdr = (Bootloader_ELFH32 *)elfHeader;

    /* Verify ELF magic number */
    if (elfHdr->e_ident[0] != 0x7F ||
        elfHdr->e_ident[1] != 'E' ||
        elfHdr->e_ident[2] != 'L' ||
        elfHdr->e_ident[3] != 'F')
    {
        DebugP_log("ERROR: Invalid ELF magic number: 0x%02X%02X%02X%02X\r\n",
                   elfHdr->e_ident[0], elfHdr->e_ident[1],
                   elfHdr->e_ident[2], elfHdr->e_ident[3]);
        return SystemP_FAILURE;
    }

    /* Check if 32-bit or 64-bit ELF */
    uint8_t elfClass = elfHdr->e_ident[ELFCLASS_IDX];

    if (elfClass == ELFCLASS_32)
    {
        /* Calculate program header table size */
        uint32_t phdrTableSize = elfHdr->e_phentsize * elfHdr->e_phnum;
        Bootloader_ELFPH32 *progHdr = (Bootloader_ELFPH32 *)malloc(phdrTableSize);

        if (progHdr == NULL)
        {
            DebugP_log("ERROR: Failed to allocate memory for program headers\r\n");
            return SystemP_FAILURE;
        }

        /* Read program header table from file */
        ff_fseek(appXipFp, elfHdr->e_phoff, FF_SEEK_SET);
        bytesRead = ff_fread(progHdr, 1, phdrTableSize, appXipFp);
        if (bytesRead < phdrTableSize)
        {
            DebugP_log("ERROR: Failed to read program headers\r\n");
            free(progHdr);
            return SystemP_FAILURE;
        }

        /* Iterate through program headers */
        for (uint32_t i = 0; i < elfHdr->e_phnum; i++)
        {
            Bootloader_ELFPH32 *phdr = &progHdr[i];

            /* Only process PT_LOAD segments with non-zero size */
            if (phdr->type == PT_LOAD && phdr->filesz > 0)
            {
                uint32_t psramOffset = phdr->paddr & 0x0FFFFFFF;  // Remove CPU region prefix
                uint32_t dataOffset = phdr->offset;
                uint32_t dataSize = phdr->filesz;
                uint32_t remainingSize = dataSize;
                uint32_t currentPsramOffset = psramOffset;

                /* Seek to segment data in file */
                ff_fseek(appXipFp, dataOffset, FF_SEEK_SET);

                /* Stream data in chunks from SD to PSRAM */
                while (remainingSize > 0)
                {
                    uint32_t chunkSize = (remainingSize < XIP_STREAMING_CHUNK_SIZE) ?
                                        remainingSize : XIP_STREAMING_CHUNK_SIZE;

                    /* Read chunk from SD card */
                    bytesRead = ff_fread(gXipStreamBuf, 1, chunkSize, appXipFp);
                    if (bytesRead < chunkSize)
                    {
                        DebugP_log("ERROR: Failed to read segment data from SD\r\n");
                        status = SystemP_FAILURE;
                        break;
                    }

                    /* Write chunk to PSRAM */
                    status = Psram_write(gPsramHandle[CONFIG_PSRAM0],
                                       currentPsramOffset,
                                       gXipStreamBuf,
                                       chunkSize);

                    if (status != SystemP_SUCCESS)
                    {
                        DebugP_log("ERROR: Failed to write to PSRAM at 0x%08X\r\n", currentPsramOffset);
                        break;
                    }

                    currentPsramOffset += chunkSize;
                    remainingSize -= chunkSize;
                }

                if (status != SystemP_SUCCESS)
                {
                    break;
                }
            }
        }

        free(progHdr);
    }
    else if (elfClass == ELFCLASS_64)
    {
        /* 64-bit ELF handling */
        Bootloader_ELFH64 *elfHdr64 = (Bootloader_ELFH64 *)elfHeader;
        uint32_t phdrTableSize = elfHdr64->e_phentsize * elfHdr64->e_phnum;
        Bootloader_ELFPH64 *progHdr = (Bootloader_ELFPH64 *)malloc(phdrTableSize);

        if (progHdr == NULL)
        {
            DebugP_log("ERROR: Failed to allocate memory for program headers\r\n");
            return SystemP_FAILURE;
        }

        /* Read program header table from file */
        ff_fseek(appXipFp, elfHdr64->e_phoff, FF_SEEK_SET);
        bytesRead = ff_fread(progHdr, 1, phdrTableSize, appXipFp);
        if (bytesRead < phdrTableSize)
        {
            DebugP_log("ERROR: Failed to read program headers\r\n");
            free(progHdr);
            return SystemP_FAILURE;
        }

        /* Iterate through program headers */
        for (uint32_t i = 0; i < elfHdr64->e_phnum; i++)
        {
            Bootloader_ELFPH64 *phdr = &progHdr[i];

            /* Only process PT_LOAD segments with non-zero size */
            if (phdr->type == PT_LOAD && phdr->filesz > 0)
            {
                uint32_t psramOffset = (uint32_t)(phdr->paddr & 0x0FFFFFFFUL);
                uint32_t dataOffset = (uint32_t)phdr->offset;
                uint32_t dataSize = (uint32_t)phdr->filesz;
                uint32_t remainingSize = dataSize;
                uint32_t currentPsramOffset = psramOffset;

                /* Seek to segment data in file */
                ff_fseek(appXipFp, dataOffset, FF_SEEK_SET);

                /* Stream data in chunks from SD to PSRAM */
                while (remainingSize > 0)
                {
                    uint32_t chunkSize = (remainingSize < XIP_STREAMING_CHUNK_SIZE) ?
                                        remainingSize : XIP_STREAMING_CHUNK_SIZE;

                    /* Read chunk from SD card */
                    bytesRead = ff_fread(gXipStreamBuf, 1, chunkSize, appXipFp);
                    if (bytesRead < chunkSize)
                    {
                        DebugP_log("ERROR: Failed to read segment data from SD\r\n");
                        status = SystemP_FAILURE;
                        break;
                    }

                    /* Write chunk to PSRAM */
                    status = Psram_write(gPsramHandle[CONFIG_PSRAM0],
                                       currentPsramOffset,
                                       gXipStreamBuf,
                                       chunkSize);

                    if (status != SystemP_SUCCESS)
                    {
                        DebugP_log("ERROR: Failed to write to PSRAM at 0x%08X\r\n", currentPsramOffset);
                        break;
                    }

                    currentPsramOffset += chunkSize;
                    remainingSize -= chunkSize;
                }

                if (status != SystemP_SUCCESS)
                {
                    break;
                }
            }
        }

        free(progHdr);
    }
    else
    {
        DebugP_log("ERROR: Unsupported ELF class: %d\r\n", elfClass);
        status = SystemP_FAILURE;
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("ERROR: XIP MCELF image programming failed!\r\n");
    }

    return status;
}

int main(void)
{
    int32_t status;

    Bootloader_profileReset();

    Bootloader_socWaitForFWBoot();

#ifndef DISABLE_WARM_REST_WA
    /* Warm Reset Workaround to prevent CPSW register lockup */
    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        Bootloader_socResetWorkaround();
    }
#endif

    Bootloader_profileAddProfilePoint("SYSFW init");

    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        /* Update devGrp to ALL to initialize MCU domain when reset isolation is
        not enabled */
        Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
            {
                .boardConfigLow = (uint32_t)0,
                .boardConfigHigh = 0,
                .boardConfigSize = 0,
                .devGrp = DEVGRP_ALL,
            };

        status = Sciclient_boardCfgPm(&boardCfgPrms_pm);

        Sciclient_BoardCfgPrms_t boardCfgPrms_rm =
        {
            .boardConfigLow = (uint32_t)0,
            .boardConfigHigh = 0,
            .boardConfigSize = 0,
            .devGrp = DEVGRP_ALL,
        };

        status = Sciclient_boardCfgRm(&boardCfgPrms_rm);

        /* Enable MCU PLL. MCU PLL will not be enabled by DMSC when devGrp is set
        to Main in boardCfg */
        Bootloader_enableMCUPLL();
    }

    System_init();
    Bootloader_profileAddProfilePoint("System_init");
    
    Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    Drivers_open();

    /* Enable the logic for switching to PSRAM*/
    status = i2c_io_expander(NULL);

    Bootloader_profileAddProfilePoint("Drivers_open");

    #if 0
    DebugP_log("\r\n");
    DebugP_log("Starting SD Bootloader ... \r\n");
    #endif

    status = Board_driversOpen();
    status = Board_psramOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    status = Sciclient_getVersionCheck(1);
    Bootloader_profileAddProfilePoint("Sciclient Get Version");
    /* File I/O */
    if(SystemP_SUCCESS == status)
    {
        /* Open app file */
        FF_FILE *appFp = ff_fopen(BOOTLOADER_SD_APPIMAGE_FILENAME, "rb");

        /* Check if file open succeeded */
        if(appFp == NULL)
        {
            status =  SystemP_FAILURE;
            DebugP_log("Failed to open app file: %s\r\n", BOOTLOADER_SD_APPIMAGE_FILENAME);
        }
        else
        {
            /* Check file size */
            uint32_t fileSize = ff_filelength(appFp);

            if(fileSize >= BOOTLOADER_APPIMAGE_MAX_FILE_SIZE)
            {
                /* Application size more than buffer size, abort */
                status = SystemP_FAILURE;
                DebugP_log("Appimage size exceeded limit !!\r\n");
            }
            else
            {
                /* Read the file into RAM buffer */
                ff_fread(gAppImageBuf, fileSize, 1, appFp);
            }

            /* Close the file */
            ff_fclose(appFp);
        }
    }
    Bootloader_profileAddProfilePoint("File read from SD card");

    /* Stream and program XIP app image directly to PSRAM */
    uint32_t xipImageSize = 0;
    if(SystemP_SUCCESS == status)
    {
        /* Open XIP app file */
        FF_FILE *appXipFp = ff_fopen(BOOTLOADER_SD_APPIMAGE_XIP_FILENAME, "rb");

        /* Check if file open succeeded */
        if(appXipFp == NULL)
        {
            DebugP_log("XIP app file not found: %s (optional)\r\n", BOOTLOADER_SD_APPIMAGE_XIP_FILENAME);
        }
        else
        {
            xipImageSize = ff_filelength(appXipFp);

            /* Stream and program directly to PSRAM */
            status = Bootloader_programXipImageStreaming(appXipFp, xipImageSize);

            if(status != SystemP_SUCCESS)
            {
                DebugP_log("Failed to program XIP image to PSRAM!\r\n");
            }

            /* Close the file */
            ff_fclose(appXipFp);
        }
    }

    Bootloader_profileAddProfilePoint("XIP image programmed to PSRAM");

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_MEM, &bootParams);
        if(bootHandle != NULL)
        {
            /* Initialize PRU Cores if applicable */
            Bootloader_Config *cfg = (Bootloader_Config *)bootHandle;
            if(TRUE == cfg->initICSSCores)
            {
                status = Bootloader_socEnableICSSCores(BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);
    
            Bootloader_profileAddProfilePoint("CPU load");
            Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
            Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_SD, 0);

            if(status == SystemP_SUCCESS)
            {
                /* Print SBL Profiling logs to UART as other cores may use the UART for logging */
                Bootloader_profileAddProfilePoint("SBL End");
                Bootloader_profilePrintProfileLog();
                DebugP_log("Image loading done, switching to application ...\r\n");
                UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
            }

            /* Run CPUs */
            /* Do not run M4 when MCU domain is reset isolated */
            if (!Bootloader_socIsMCUResetIsoEnabled())
            {
                if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
                {
                    status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
                }
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
            }
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}
