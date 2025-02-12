/*
 *  Copyright(C) 2023 Texas Instruments Incorporated
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
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <string.h>
#include <board/psram.h>
#include "psram_ospi.h"
#include <drivers/hw_include/cslr.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_open_close.h"
#include "fss_config.h"
#include "ospi_psram_aps6408.h"

Psram_Fxns gPsramOspiFxns = {
    .openFxn = Psram_ospiOpen,
    .closeFxn = Psram_ospiClose,
    .readFxn = Psram_ospiRead,
    .writeFxn = Psram_ospiWrite,
};

int32_t Psram_ospiOpen(Psram_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t manfId, deviceId;
    Psram_OspiObject *obj = (Psram_OspiObject*)(config->object);

    /* Wait for a while */
    uint32_t waitMicro = 500U * 1000U;
    ClockP_usleep(waitMicro);

    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    const OSPI_Attrs *ospi_attrs = ((OSPI_Config *)obj->ospiHandle)->attrs;
    volatile CSL_ospi_flash_cfgRegs *pReg = (volatile CSL_ospi_flash_cfgRegs *)(ospi_attrs->baseAddr);
    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
         OSPI_configResetPin(obj->ospiHandle, OSPI_RESETPIN_DEDICATED);

         /* Set device size and addressing bytes */
         OSPI_setDeviceSize(obj->ospiHandle, PAGE_SIZE_8MB, BLOCK_SIZE_8MB);


         //////////////////////Using config reg directly///////////////////////////////////////////////////////

         ospi_stig_cmd_ctrl(pReg,STIG_MEM_BANK_DIS,MODE_BIT_DIS,DUMMY_CYC0,ADDR_BYTE_CNT4,CMD_ADDR_EN ); //Address size 4bytes
         ospi_stig_cmd_ctrl_data_cfg(pReg,WR_CMD,DATA_BYTE_CNT1);

         ospi_read_instr_cfg(pReg,OCTAL_SPI_MODE,DTR_PROTOCOL_EN,MODE_BIT_DIS,DUMMY_CYC5,PHY_DIS);
         ospi_wr_instr_cfg(pReg,AUTO_WEL_EN,DUMMY_CYC2);
         ospi_poll_status_from_flash_cfg(pReg,POLL_DLY0,POLL_CNT0,POLL_DIS,POLL_POL_LOW,POLL_BIT0,DBOPCODE,DUMMY_CYC0 );

         ospi_cs_dly_cfg(pReg,0x0,0x0,0x1,0x1 ); //CS delay is also done in Drivers_ospiOpen() -> OSPI_open() -> OSPI_programInstance() -> devDelay -> { 10, 10, 10, 10 };. Do we need to redo it here?

         //Summary: Configuring for Fixed Latency= 4Cyles, 16Byte Burst length
         uint32_t MR0 = OSPI_PSRAM_APS6408_MR0_DRIVE_STRENGTH + OSPI_PSRAM_APS6408_MR0_RLC_3 + OSPI_PSRAM_APS6408_MR0_LATENCY_TYPE;    // Fixed Latency, Drive Strength =3, Read Latency Code =4
         uint32_t MR4 = OSPI_PSRAM_APS6408_MR4_WLC_3;    // Write Latency Code =4
         uint32_t MR8 = OSPI_PSRAM_APS6408_MR8_BT;    // 16 Byte Hybrid Wrap

         ospi_stig_cmd_addr(pReg,OSPI_PSRAM_APS6408_MR0_ADDRESS);
         ospi_stig_cmd_wr_data_lower(pReg,MR0);

         ospi_stig_cmd_opcode(pReg,DB_OPCODE_MODE_DIS,OSPI_PSRAM_APS6408_WRITE_REG_CMD,0x0);

         ospi_stig_cmd_addr(pReg,OSPI_PSRAM_APS6408_MR4_ADDRESS);
         ospi_stig_cmd_wr_data_lower(pReg,MR4);

         ospi_stig_cmd_opcode(pReg,DB_OPCODE_MODE_DIS,OSPI_PSRAM_APS6408_WRITE_REG_CMD,0x0);
         ospi_stig_cmd_wait_for_done(pReg);


         ospi_stig_cmd_addr(pReg,OSPI_PSRAM_APS6408_MR8_ADDRESS);
         ospi_stig_cmd_wr_data_lower(pReg,MR8);

         ospi_stig_cmd_opcode(pReg,DB_OPCODE_MODE_DIS,OSPI_PSRAM_APS6408_WRITE_REG_CMD,0x0);
         ospi_stig_cmd_wait_for_done(pReg);
         ospi_cs_dly_cfg(pReg,0x4,0x0,0x5,0x1);
         CSL_REG32_WR(0x0fc40014, 0x00101003); //Set num of addr bytes to 4 in DEV_SIZE reg.
         ///////////////////////////////////////////////////////////////////////////////////////////////////////////

        /* Wait for a while */
        uint32_t waitMicro = 500U * 1000U;
        ClockP_usleep(waitMicro);
    }
	OSPI_setRdDataCaptureDelay(obj->ospiHandle, 0x5);

    /* Read ID */
    status += Psram_ospiReadId(config, &manfId, &deviceId);

    /* Compare Manufacturer ID and Device ID For APS6804 */
    if((SystemP_SUCCESS == status) && (deviceId == OSPI_PSRAM_APS6408_MR2_DEVID_GEN_3) && (manfId == OSPI_PSRAM_APS6408_MR1_VENDOR_ID_APM))
    {
        DebugP_log("PSRAM Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("PSRAM Device ID: 0x%X\r\n", deviceId);

    }
    else
    {
        DebugP_log("Unsuccessfully connect to PSRAM!!\r\n");
        DebugP_log("Read PSRAM Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("Read PSRAM Device ID: 0x%X\r\n", deviceId);
    }

    /* Set RD and WR Config register */
    if(obj->syncModeEnable)
    {
        ospi_inst_opcode(pReg,DBOPCODE,OSPI_PSRAM_APS6408_READ_CMD,0x00,OSPI_PSRAM_APS6408_WRITE_CMD,0x00);
    }
    else
    {
        ospi_inst_opcode(pReg,DBOPCODE,OSPI_PSRAM_APS6408_READ_LINEAR_BURST_CMD,0x00,OSPI_PSRAM_APS6408_WRITE_LINEAR_BURST_CMD,0x00);
    }
    /* Set read capture delay */
    status += OSPI_setRdDataCaptureDelay(obj->ospiHandle, 0x5);
    return status;
}


int32_t Psram_ospiWrite(Psram_Config *config, uint32_t offset, uint8_t *pbuf, uint32_t len)

{
    int32_t status = SystemP_SUCCESS;

    Psram_Attrs *attrs = config->attrs;
    Psram_OspiObject *obj = (Psram_OspiObject*)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    const OSPI_Attrs *ospiAttrs = ((OSPI_Config *)obj->ospiHandle)->attrs;
    volatile CSL_ospi_flash_cfgRegs *pReg = (volatile CSL_ospi_flash_cfgRegs *)(ospiAttrs->baseAddr);
    ospi_dac_en(pReg,obj->dacEnable); //Update the mode of transfer
    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    /* Validate address input */
    if((offset + len) > (attrs->psramSize))
    {
        status = SystemP_FAILURE;
    }

    /*From datasheet APS6408L we have:
    /*  Memory accesses are required to start on even addresses
    /*  Write burst length has a minimum of 2 bytes. */
    if(len < 2)
    {
        status = SystemP_FAILURE;
    }

    /* check if writeAddr is even, if not fix it */
    if(offset & 1)
    {
        offset--;
    }
    if (status == SystemP_SUCCESS)
    {
        if(obj->dacEnable == FALSE)
        {
            for(int c=0; c<len;c+=512)
            {
                OSPI_Transaction transaction;
                OSPI_Transaction_init(&transaction);
                transaction.addrOffset = offset+c;
                transaction.buf = (void *)(pbuf+c);
                transaction.count = 512;
                status = OSPI_writeIndirect(obj->ospiHandle, &transaction);
            }
        }
        else
        {
            OSPI_Transaction transaction;
            OSPI_Transaction_init(&transaction);
            transaction.addrOffset = offset;
            transaction.buf = (void *)(pbuf);
            transaction.count = len;
            status = OSPI_writeDirect(obj->ospiHandle, &transaction);
        }
    }
    uint32_t waitMicro = 500U * 1000U;
	ClockP_usleep(waitMicro);
    return status;
}

int32_t Psram_ospiRead(Psram_Config *config, uint32_t offset, uint8_t *pbuf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    Psram_Attrs *attrs = config->attrs;
    Psram_OspiObject *obj = (Psram_OspiObject*)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
    const OSPI_Attrs *ospiAttrs = ((OSPI_Config *)obj->ospiHandle)->attrs;
    volatile CSL_ospi_flash_cfgRegs *pReg = (volatile CSL_ospi_flash_cfgRegs *)(ospiAttrs->baseAddr);
    
    ospi_dac_en(pReg,obj->dacEnable); //Update the mode of transfer  
    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    /* Validate address input */
    if ((offset + len) > (attrs->psramSize))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        OSPI_Transaction transaction;
        OSPI_Transaction_init(&transaction);
        transaction.addrOffset = offset;
        transaction.buf = (void *)pbuf;
        transaction.count = len;
        if(obj->dacEnable == FALSE)
        {
            status = OSPI_readIndirect(obj->ospiHandle, &transaction);
        }
        else
        {
            status = OSPI_readDirect(obj->ospiHandle, &transaction);
        }
    }
    return status;
}

int32_t Psram_ospiReadCmd(Psram_Config *config, uint8_t cmd, uint32_t ReadAddr, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;
    Psram_OspiObject *obj = (Psram_OspiObject*)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    OSPI_ReadCmdParams  rdParams;

    if(status == SystemP_SUCCESS)
    {
        OSPI_ReadCmdParams_init(&rdParams);
        rdParams.cmd           = cmd;
        rdParams.cmdAddr       = ReadAddr;
        rdParams.rxDataBuf     = rxBuf;
        rdParams.rxDataLen     = rxLen;
        rdParams.dummyBits     = 5;
		rdParams.numAddrBytes  = 4;
		
        status = OSPI_readCmd(obj->ospiHandle, &rdParams);
    }

    return status;
}

int32_t Psram_ospiReadId(Psram_Config *config, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmd = OSPI_PSRAM_APS6408_READ_REG_CMD;
    uint32_t cmdAddr = OSPI_PSRAM_APS6408_MR1_ADDRESS;
    uint8_t idCode[2] = { 0 };

    status +=Psram_ospiReadCmd(config, cmd, cmdAddr, idCode, 2);

    if(status == SystemP_SUCCESS)
    {
        // Keep only Vendor ID from Mode Register 1 */
        *manufacturerId = (uint32_t)idCode[0] & (OSPI_PSRAM_APS6408_MR1_VENDOR_ID);
        // Keep only Device ID and Device Density from Mode Register 2 */
		*deviceId = ((uint32_t)idCode[1] & (OSPI_PSRAM_APS6408_MR2_DEVICE_ID)) >> (3);
    }

    return status;
}

int32_t Psram_ospiWriteCmd(Psram_Config *config, uint8_t cmd, uint32_t writeAddr, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;
    Psram_OspiObject *obj = (Psram_OspiObject *)(config->object);
    obj->ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        OSPI_WriteCmdParams wrParams;
        OSPI_WriteCmdParams_init(&wrParams);
        wrParams.cmd        = cmd;
        wrParams.cmdAddr    = writeAddr;
        wrParams.txDataBuf  = txBuf;
        wrParams.txDataLen  = txLen;
        status += OSPI_writeCmd(obj->ospiHandle , &wrParams);
    }

    return status;
}

int32_t Psram_ospiReset(Psram_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    status = Psram_ospiWriteCmd(config, OSPI_PSRAM_APS6408_RESET_CMD, 0, NULL, 0);
    /* Need to wait tRST */
    uint32_t waitMicro = 1000000U;
    ClockP_usleep(waitMicro);

    return status;
}

void Psram_ospiClose(Psram_Config *config)
{
    return;
}
