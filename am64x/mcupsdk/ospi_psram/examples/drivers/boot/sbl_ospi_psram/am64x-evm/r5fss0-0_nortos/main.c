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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_elf.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <board/psram.h>
#include "ti_board_psram_open_close.h"
#include "ospi_psram_config.h"

/*
 * Flash offset where the XIP ELF image is stored.
 * The flashing config must use:
 *   --file=.mcelf_xip --operation=flash --flash-offset=0xB00000
 */
#define XIP_FLASH_OFFSET    (0xB00000U)

/*
 * Staging buffer size for the XIP ELF image (320 KB). 
 * This value can be changed as per application requirement.
 * The entire XIP ELF is read from flash into this buffer before switching
 * the OSPI to PSRAM mode, to transfer the contents of the buffer to PSRAM.
 */
#define XIP_STAGING_SIZE    (0x50000U)

/* XIP staging buffer placed in MSRAM */
uint8_t gXipStagingBuf[XIP_STAGING_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

extern Psram_Config gPsramConfig[];

/* Save original flash OSPI attrs pointer for switching back later */
static const OSPI_Attrs *gOspiAttrs_Flash = NULL;

void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

/*
 * Parse the XIP ELF already staged in gXipStagingBuf and write each
 * PT_LOAD segment to PSRAM. PSRAM must be open before calling this.
 */
static int32_t Bootloader_writeXipStagingToPsram(void)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t *elfBuf = gXipStagingBuf;

    Bootloader_ELFH32 *elfHdr = (Bootloader_ELFH32 *)elfBuf;

    if (elfHdr->e_ident[0] != 0x7F || elfHdr->e_ident[1] != 'E' ||
        elfHdr->e_ident[2] != 'L' || elfHdr->e_ident[3] != 'F')
    {
        DebugP_log("ERROR: Invalid ELF magic in XIP staging buffer\r\n");
        return SystemP_FAILURE;
    }

    uint8_t elfClass = elfHdr->e_ident[ELFCLASS_IDX];

    if (elfClass == ELFCLASS_32)
    {
        Bootloader_ELFPH32 *progHdrs =
            (Bootloader_ELFPH32 *)(elfBuf + elfHdr->e_phoff);

        for (uint32_t i = 0; i < elfHdr->e_phnum; i++)
        {
            Bootloader_ELFPH32 *phdr = &progHdrs[i];
            if (phdr->type == PT_LOAD && phdr->filesz > 0)
            {
                uint32_t psramOffset = phdr->paddr & 0x0FFFFFFFUL;
                status = Psram_write(gPsramHandle[CONFIG_PSRAM0],
                                     psramOffset,
                                     elfBuf + phdr->offset,
                                     phdr->filesz);
                if (status != SystemP_SUCCESS)
                {
                    DebugP_log("ERROR: PSRAM write failed at offset 0x%08X\r\n",
                               psramOffset);
                    return status;
                }
            }
        }
    }
    else if (elfClass == ELFCLASS_64)
    {
        Bootloader_ELFH64 *elfHdr64 = (Bootloader_ELFH64 *)elfBuf;
        Bootloader_ELFPH64 *progHdrs =
            (Bootloader_ELFPH64 *)(elfBuf + (uint32_t)elfHdr64->e_phoff);

        for (uint32_t i = 0; i < elfHdr64->e_phnum; i++)
        {
            Bootloader_ELFPH64 *phdr = &progHdrs[i];
            if (phdr->type == PT_LOAD && phdr->filesz > 0)
            {
                uint32_t psramOffset = (uint32_t)(phdr->paddr & 0x0FFFFFFFUL);
                status = Psram_write(gPsramHandle[CONFIG_PSRAM0],
                                     psramOffset,
                                     elfBuf + (uint32_t)phdr->offset,
                                     (uint32_t)phdr->filesz);
                if (status != SystemP_SUCCESS)
                {
                    DebugP_log("ERROR: PSRAM write failed at offset 0x%08X\r\n",
                               psramOffset);
                    return status;
                }
            }
        }
    }
    else
    {
        DebugP_log("ERROR: Unsupported ELF class: %d\r\n", elfClass);
        status = SystemP_FAILURE;
    }

    return status;
}

int main(void)
{
    int32_t status;

    Bootloader_profileReset();

    Bootloader_socWaitForFWBoot();

#ifndef DISABLE_WARM_REST_WA
    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        Bootloader_socResetWorkaround();
    }
#endif

    Bootloader_profileAddProfilePoint("SYSFW init");

    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
        {
            .boardConfigLow  = (uint32_t)0,
            .boardConfigHigh = 0,
            .boardConfigSize = 0,
            .devGrp          = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgPm(&boardCfgPrms_pm);

        Sciclient_BoardCfgPrms_t boardCfgPrms_rm =
        {
            .boardConfigLow  = (uint32_t)0,
            .boardConfigHigh = 0,
            .boardConfigSize = 0,
            .devGrp          = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgRm(&boardCfgPrms_rm);

        Bootloader_enableMCUPLL();
    }

    System_init();
    Bootloader_profileAddProfilePoint("System_init");

    Bootloader_socOpenFirewalls();
    Bootloader_socNotifyFirewallOpen();

    /* Drivers_open() opens OSPI (flash config: CS0, 166 MHz), I2C, UART */
    Drivers_open();

    /* Ensure the hardware switch routes OSPI to flash (Switch is set to LOW) */
    status = i2c_io_expander_set2flash(NULL);
    DebugP_assert(status == SystemP_SUCCESS);

    Bootloader_profileAddProfilePoint("Drivers_open");

    /* Board_driversOpen() opens the S28HS512T flash layer */
    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    status = Sciclient_getVersionCheck(1);
    Bootloader_profileAddProfilePoint("Sciclient Get Version");

    if (SystemP_SUCCESS == status)
    {
        /*
         * Step 1: Read the entire XIP ELF image from flash into the staging
         * buffer. Flash remains open after this call.
         */
        status = Flash_read(gFlashHandle[CONFIG_FLASH0],
                            XIP_FLASH_OFFSET,
                            gXipStagingBuf,
                            XIP_STAGING_SIZE);
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("ERROR: Failed to read XIP image from flash "
                       "at offset 0x%08X\r\n", XIP_FLASH_OFFSET);
        }
        Bootloader_profileAddProfilePoint("XIP staged from flash");
    }

    if (SystemP_SUCCESS == status)
    {
        /*
         * Step 2: Flash reading is done. Close the flash layer and the
         * OSPI driver, then reconfigure OSPI for PSRAM (CS1, 200/8 MHz).
         * Save the original flash attrs pointer before switching.
         */
        /* Save original flash attrs pointer */
        gOspiAttrs_Flash = gOspiConfig[CONFIG_OSPI0].attrs;

        Board_flashClose();
        Drivers_ospiClose();

        gOspiConfig[CONFIG_OSPI0].attrs = (const OSPI_Attrs *)&gOspiAttrs_Psram;

        /* Set OSPI clock to 200 MHz for PSRAM before opening driver.
         * PowerClock_init() set it to 166.67 MHz for flash, and it stays at that
         * frequency unless we explicitly change it via TISCI/Sciclient. */
        status = SOC_moduleSetClockFrequency(TISCI_DEV_FSS0_OSPI_0,
                                              TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK,
                                              200000000U);
        DebugP_assert(status == SystemP_SUCCESS);

        Drivers_ospiOpen();

        /* Toggle hardware switch HIGH: route OSPI bus to PSRAM */
        status = i2c_io_expander(NULL);
        DebugP_assert(status == SystemP_SUCCESS);

        /*
         * Step 3: Open PSRAM driver and write XIP data from the staging
         * buffer into PSRAM.
         */
        status = Board_psramOpen();
        DebugP_assert(status == SystemP_SUCCESS);

        status = Bootloader_writeXipStagingToPsram();
        if (status != SystemP_SUCCESS)
        {
            DebugP_log("ERROR: Failed to write XIP image to PSRAM\r\n");
        }
        Bootloader_profileAddProfilePoint("XIP written to PSRAM");
    }

    if (SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params        bootParams;
        Bootloader_Handle        bootHandle;
        Bootloader_Config       *bootConfig;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        /*
         * Step 4: Load the main app image from Flash (OSPI configured for Flash).
         * Now the flash-based bootloader opens and loads the app from flash,
         * but the OSPI has been reconfigured to PSRAM mode, so we
         * close PSRAM temporarily to reopen OSPI in flash mode */
        Board_psramClose();
        Drivers_ospiClose();

        /* Reconfigure OSPI back to flash mode using saved attrs pointer */
        gOspiConfig[CONFIG_OSPI0].attrs = gOspiAttrs_Flash;

        /* Set OSPI clock back to 166.67 MHz for flash before opening driver */
        status = SOC_moduleSetClockFrequency(TISCI_DEV_FSS0_OSPI_0,
                                              TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK,
                                              166666666U);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Reopen OSPI for flash */
        Drivers_ospiOpen();

        /* Set hardware switch to flash */
        status = i2c_io_expander_set2flash(NULL);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Reopen flash driver */
        status = Board_driversOpen();
        DebugP_assert(status == SystemP_SUCCESS);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_FLASH0, &bootParams);
        if (bootHandle != NULL)
        {
            bootConfig = (Bootloader_Config *)bootHandle;
            bootConfig->scratchMemPtr = NULL;

            if (TRUE == bootConfig->initICSSCores)
            {
                status = Bootloader_socEnableICSSCores(
                             BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            /* Initialize M4F core to make its memory accessible, even though we won't run M4F code, we are only loading the sections to M4F_IRAM */
            if (!Bootloader_socIsMCUResetIsoEnabled())
            {
                /* Request M4F from DMSC */
                status = Bootloader_socCpuRequest(CSL_CORE_ID_M4FSS0_0);
                if(status == SystemP_SUCCESS)
                {
                    /* Set M4F clock to default frequency */
                    status = Bootloader_socCpuSetClock(CSL_CORE_ID_M4FSS0_0, 400000000); // 400 MHz
                }
                if(status == SystemP_SUCCESS)
                {
                    /* Power on M4F and initialize its IRAM - this makes 0x05000000 accessible */
                    status = Bootloader_socCpuPowerOnReset(CSL_CORE_ID_M4FSS0_0, NULL);
                }
                DebugP_assert(status == SystemP_SUCCESS);
            }

            status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);

            Bootloader_profileAddProfilePoint("CPU Load");
            Bootloader_profileUpdateMediaAndClk(
                BOOTLOADER_MEDIA_FLASH,
                OSPI_getInputClk(gOspiHandle[CONFIG_OSPI0]));

            if (status == SystemP_SUCCESS)
            {
                /*
                 * Step 5: App image loaded. Now switch back to PSRAM mode
                 * for the running application.
                 */
                Board_flashClose();
                Drivers_ospiClose();

                gOspiConfig[CONFIG_OSPI0].attrs = (const OSPI_Attrs *)&gOspiAttrs_Psram;

                /* Set OSPI clock to 200 MHz for PSRAM before opening driver.
                 * Must do this again after switching back from flash mode. */
                status = SOC_moduleSetClockFrequency(TISCI_DEV_FSS0_OSPI_0,
                                                      TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK,
                                                      200000000U);
                DebugP_assert(status == SystemP_SUCCESS);

                Drivers_ospiOpen();

                /* Toggle hardware switch HIGH: route OSPI bus to PSRAM */
                status = i2c_io_expander(NULL);
                DebugP_assert(status == SystemP_SUCCESS);

                /* Reopen PSRAM driver - it stays open for the running application */
                status = Board_psramOpen();
                DebugP_assert(status == SystemP_SUCCESS);

            }

            if (status == SystemP_SUCCESS)
            {
                /* Step 6: Run application cores. PSRAM and OSPI drivers remain open. */
                if (!Bootloader_socIsMCUResetIsoEnabled())
                {
                    if (TRUE == Bootloader_isCorePresent(bootHandle,
                                                         CSL_CORE_ID_M4FSS0_0))
                    {
                        status = Bootloader_runCpu(
                                     bootHandle,
                                     &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
                    }
                }
                if (status == SystemP_SUCCESS &&
                    TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0))
                {
                    status = Bootloader_runCpu(
                                 bootHandle,
                                 &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
                }
                if (status == SystemP_SUCCESS &&
                    TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1))
                {
                    status = Bootloader_runCpu(
                                 bootHandle,
                                 &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
                }
                if (status == SystemP_SUCCESS &&
                    TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0))
                {
                    status = Bootloader_runCpu(
                                 bootHandle,
                                 &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
                }
                if (status == SystemP_SUCCESS &&
                    TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_1))
                {
                    status = Bootloader_runCpu(
                                 bootHandle,
                                 &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
                }
                if (status == SystemP_SUCCESS)
                {
                    Bootloader_profileUpdateAppimageSize(
                        Bootloader_getMulticoreImageSize(bootHandle));
                    Bootloader_profileAddProfilePoint("SBL End");
                    Bootloader_profilePrintProfileLog();
                    DebugP_log("Image loading done, switching to application ...\r\n");
                    UART_flushTxFifo(gUartHandle[CONFIG_UART0]);

                    status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
                }
            }

            /* Reached only on error */
            Bootloader_close(bootHandle);
        }
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}
