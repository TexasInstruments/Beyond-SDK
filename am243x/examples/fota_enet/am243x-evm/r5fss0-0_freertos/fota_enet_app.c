/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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
#include <string.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "kernel/dpl/CacheP.h"
#include "ti_drivers_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board/flash.h"
#include "drivers/bootloader/soc/am64x_am243x/bootloader_soc.h"
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include "drivers/bootloader.h"
#include "fota_enet.h"




#define APP_IMAGE_OFFSETA (0x80000U) // Offsets used for the App Images
#define APP_IMAGE_OFFSETB (0x900000U) // Offsets used for the App Images 
#define FLAG_OFFSET (0x1000000U) // Offset for Flag

int32_t APPIMG_FILE_LEN; 
int32_t NUMBER_OF_PACKETS;

#define CONFIG_FLASH0 (0U)
#define CONFIG_FLASH_NUM_INSTANCES (1U)
Flash_Handle gFlashHandle[CONFIG_FLASH_NUM_INSTANCES];

uint8_t gOspiRxBuf[MAX_FILE_SIZE] __attribute__((aligned(128))) __attribute__((section(".ddr_memory")));
uint8_t gOspiFlagBuf[2] __attribute__((aligned(128U))); // Flash Writes work only if Buffer size is even numbered

uint32_t receiveAppImgOverEnet();
void ospi_flash_io_fill_buffers(void);
int32_t ospi_flash_io_compare_buffers(void);

Float32 timeTaken1, timeTaken2, timeTaken3, timeTaken4;     

void fota_enet_main(void *args)
{

    int32_t status = SystemP_SUCCESS;
    int32_t authStatus = SystemP_FAILURE;
    uint32_t offset;
    uint32_t offset1,offset2;
    uint32_t flagoffset;
    uint32_t flag;
    uint32_t blk, page;
    uint32_t certLoadAddr = 0xFFFFFFFFU;
    uint32_t certLen = 0U;
    uint32_t imageLen = 0U;
    uint32_t time1 = 0U,time2 = 0U; 
    uint32_t numberBlocksToErase = 0U;

    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    offset1 = APP_IMAGE_OFFSETA;
    offset2 = APP_IMAGE_OFFSETB;
    flagoffset = FLAG_OFFSET;

    Flash_Config *config = (Flash_Config*)gFlashHandle[CONFIG_FLASH0];
    uint32_t blockSize  = config->attrs->blockSize;

    DebugP_log("\r\nApp Image A! (Offset: 0x80000)\r\n");

    DebugP_log("\r\nListening for new app image\r\n");  

    /* Receive application image via UDP over ethernet */
    status = receiveAppImgOverEnet();

    // Remove the first 32 Bytes of the Buffer, since its not part of the App Image
    // Forst 32 Bytes contains Magic Number and Sequence Number
    memmove(gFlashFileBuf, gFlashFileBuf + 32, MAX_FILE_SIZE - 32);
    memset(gFlashFileBuf + MAX_FILE_SIZE - 32, 0, 32);


    if(APPIMG_FILE_LEN > 0 && NUMBER_OF_PACKETS > 0)
    {
        //measure time
        time1 = ClockP_getTimeUsec();

        // Image Authentication
        certLoadAddr = (uint32_t)gFlashFileBuf;

        certLen = Bootloader_getX509CertLen((uint8_t *)certLoadAddr);
        imageLen = Bootloader_getMsgLen((uint8_t *)certLoadAddr, certLen);
        uint32_t cacheAlignedLen = (certLen + imageLen + 128) & ~(127);

        CacheP_wbInv((void *)certLoadAddr, cacheAlignedLen, CacheP_TYPE_ALL);

        authStatus = Bootloader_socAuthImage(certLoadAddr);

        time2 = ClockP_getTimeUsec();
        timeTaken3 = time2 - time1;
        if(SystemP_SUCCESS != authStatus)
        {
            DebugP_logError("\r\nImage Authentication Failed\r\n");
        }
        else
        {
            DebugP_log("\r\nImage Authentication Successful\r\n");
        }
    
        // Remove the extra certificate which was added before Enet Transfer
        memmove(gFlashFileBuf, gFlashFileBuf + certLen, MAX_FILE_SIZE - 32 - certLen);
        memset(gFlashFileBuf + MAX_FILE_SIZE - 32 - certLen, 0, certLen);


        // Checking the Flag
        status = Flash_read(gFlashHandle[CONFIG_FLASH0], flagoffset, gOspiFlagBuf, 1);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("\r\nFlash Read of %d bytes failed at 0x%X offset\r\n", 1, flagoffset);
        }

        flag = gOspiFlagBuf[0];
        if(flag == 9) 
        {
            offset = offset1; // New App Image will be written to 0x80000
        }
        else
        {
            offset = offset2; // New App Image will be written to 0x900000
        }

        // Flashing New App Image to Flash Memory
        if(SystemP_SUCCESS == authStatus)
        {
            // measure time
            time1 = ClockP_getTimeUsec();

            // Erasing reuquired number of blocks
            numberBlocksToErase = ceil((double_t)APPIMG_FILE_LEN/blockSize);
            Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
            while(numberBlocksToErase)
            {

                status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
                if(SystemP_SUCCESS != status)
                {
                    DebugP_log("\r\nBlock Erase Failed at 0x%X offset\r\n", offset + blk*262144);
                }
                blk++;
                numberBlocksToErase--;
            }

            // Flashing New App Image into the Flash memory using Flash Write
            if(SystemP_SUCCESS == status)
            {

                status = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, gFlashFileBuf, APPIMG_FILE_LEN);
                time2 = ClockP_getTimeUsec();
                timeTaken4 = time2 - time1;
            
                if(SystemP_SUCCESS != status)
                {
                    DebugP_log("\r\nFlash Write of %d bytes failed at 0x%X offset\r\n", APPIMG_FILE_LEN, offset);
                }
            }
            DebugP_log("\r\nFlashed New App Image\r\n");

            // Update the Flag
            if(flag == 9)
            {
                gOspiFlagBuf[0] = 8;
            }
            else
            {
                gOspiFlagBuf[0] = 9;
            }

            Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], flagoffset, &blk, &page);
            status = Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\nBlock Erase Failed at 0x%X offset\r\n", flagoffset);
            }

            status = Flash_write(gFlashHandle[CONFIG_FLASH0], flagoffset, gOspiFlagBuf, 1);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\nFlash Write of %d bytes failed at 0x%X offset\r\n", 1, flagoffset);
            }

            DebugP_log("\r\n--- Time Profile (in seconds) ---\r\n");
            DebugP_log("Time Taken for Ethernet Setup:               %.6fs\r\n",timeTaken1/1000000);
            DebugP_log("Time Taken for Transferring New App Image:   %.6fs\r\n",timeTaken2/1000000);
            DebugP_log("Time Taken for Authenticating New App Image: %.6fs\r\n",timeTaken3/1000000);
            DebugP_log("Time Taken for Flashing New App Image:       %.6fs\r\n",timeTaken4/1000000);
        
            // Trigger a SOC Reset
            DebugP_log("\r\nSOC Reset\r\n");
            SOC_generateSwWarmResetMcuDomain();

        }
    }

    Board_driversClose();
    Drivers_close();
}

uint32_t receiveAppImgOverEnet()
{
    int32_t status = SystemP_SUCCESS;
    uint8_t done = false;
    uint32_t time1 = 0U,time2 = 0U; 
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;
    Bootloader_UniflashFileHeader *pktInfo;

    if(!done)
    {
        DebugP_log("\r\n[ ENETFOTA ] Starting Ethernet Transfer ...\r\n");


        /* Initialize fota_enet config and setup ethernet peripheral */
        //measure time
        time1 = ClockP_getTimeUsec();
        memset(&gEnetFOTA_LLDObj, 0, sizeof(gEnetFOTA_LLDObj));
        memset(&gEnetFOTA_MetaObj, 0, sizeof(gEnetFOTA_MetaObj));
        memset(&respHeader, 0, sizeof(respHeader));
        memset(&uniflashConfig, 0, sizeof(uniflashConfig));

        status = EnetFOTA_setup();

        if(status == ENET_SOK)
        {
            /* Send ACK packet to let host know that EVM is linked up */
            respHeader.magicNumber = ENETFOTA_HEADER_MGC_NUMBER;
            respHeader.statusCode = ENETFOTA_HEADER_ACK;
            EnetFOTA_txFlashResp(respHeader);

            time2 = ClockP_getTimeUsec();
            timeTaken1 = time2 - time1;
        }
        else if(status == ENET_ETIMEOUT)
        {
            DebugP_log("[ ENETFOTA TIMEOUT ] Link Up Timeout. Please check ethernet cable connections.\r\n");
            done = true;
            status = SystemP_TIMEOUT;
        }

        while (!done)
        {
            //measure time
            time1 = ClockP_getTimeUsec();

            status = EnetFOTA_transferAppimage();

            time2 = ClockP_getTimeUsec();
            timeTaken2 = time2 - time1;
            if(gFlashFileSize >= MAX_FILE_SIZE)
            {
                /* Possible overflow, send error to host side */
                status = SystemP_FAILURE;

                respHeader.magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER;
                respHeader.statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;

                EnetFOTA_txFlashResp(respHeader);

                /* Exit due to possible error */
                done = 1U;
                DebugP_log("[ ENETFOTA ERROR ] Overflow detected.\r\n");
                break;
            }

            if(status == ENET_SOK)
            {
                    pktInfo = (Bootloader_UniflashFileHeader*) &gFlashFileBuf;
                    status = EnetFOTA_txFlashResp(respHeader);
                    DebugP_log("[ ENETFOTA SUCCESS ] Ethernet Transfer Done.\r\n");
                    DebugP_log("[ ENETFOTA ] Packets Received   :  %d \r\n",(pktInfo->rsv1-1));
                    DebugP_log("[ ENETFOTA ] Total File Size    :  %d Bytes\r\n",pktInfo->actualFileSize);
                    status = SystemP_SUCCESS;
                    NUMBER_OF_PACKETS = pktInfo->rsv1-1;
                    APPIMG_FILE_LEN = pktInfo->actualFileSize - 32 - ((NUMBER_OF_PACKETS)*8);
                    break;
            }
            else
            {
                DebugP_log("[ ENETFOTA TIMEOUT ] Skipping enet transfer.\r\n");
                status = SystemP_FAILURE;
                break;
            }
        }

        /* Close */
        EnetFOTA_destruct();
    }
    return status;
}




