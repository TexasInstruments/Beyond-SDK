/**
 * @file i2c_api.c
 * @brief I2C API implementation for TI AM2432 Board Diagnostic project
 * @author TI Board Diagnostic Team
 * @version 1.0
 * @date 2026
 * 
 * This file implements I2C communication APIs including configuration,
 * register read/write operations, multi-register operations, and raw data
 * transfer for the Board Diagnostic system.
 */

#include "../header_files/common_extern.h"
#include "../header_files/uart_cmd_header.h"

/* I2C address offsets for different domains */
#define SOC_I2C0_ADDRESS_OFFSET   0
#define MCU_I2C0_ADDRESS_OFFSET   0
#define SINGLE_REG_DATA_LEN       2

/* External I2C driver objects and handles */
extern I2CLLD_Object gI2cLldObjects[CONFIG_I2C_LLD_NUM_INSTANCES];
extern I2CLLD_Handle gI2cLldHandle[CONFIG_I2C_LLD_NUM_INSTANCES];

/* User configuration array for I2C instances */
USER_CONFIG_PARA i2c_userConfig[CONFIG_I2C_LLD_NUM_INSTANCES];

/**
 * @brief Pinmux configuration for MCU I2C0
 * Configures SCL and SDA pins for I2C operation
 */
static Pinmux_PerCfg_t gI2CPinMcuDomainCfg[] = {
    /* MCU_I2C0_SCL pin configuration -> MCU_I2C0_SCL (E9) */
    {
        PIN_MCU_I2C0_SCL,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* MCU_I2C0_SDA pin configuration -> MCU_I2C0_SDA (A10) */
    {
        PIN_MCU_I2C0_SDA,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
          
    {PINMUX_END, PINMUX_END}
};

/**
 * @brief Configure I2C channel with specified speed
 * @param ch I2C channel number 
 * @param speed I2C communication speed
 * @return SystemP_SUCCESS on success, error code on failure
 */
int I2C_Config(I2C_Channel ch, I2C_Speed speed)
{
    switch (ch) 
    {
        case MCU_I2C0_CH0:
        {
            ti_status = SystemP_SUCCESS;

            /* Configure pin mux for MCU_I2C0_CH0 */
            Pinmux_config(gI2CPinMcuDomainCfg, PINMUX_DOMAIN_ID_MCU);

            /* Close any previously opened driver */
            I2C_lld_deInit(gI2cLldHandle[MCU_I2C0_CH0]);
            
            /* Clear I2C configuration flag */
            gUARTCmdObjects.uarti2cparam.uI2CHandle = 0;

            /* Update I2C configuration with new speed */
            gI2cLldObjects[MCU_I2C0_CH0].bitRate = speed;

            /* Initialize and open I2C driver */
            gI2cLldHandle[MCU_I2C0_CH0] = &gI2cLldObjects[MCU_I2C0_CH0];
            ti_status += I2C_lld_init(gI2cLldHandle[MCU_I2C0_CH0]);

            if (ti_status != SystemP_SUCCESS)
            {
               /* Close driver if initialization fails */
               I2C_lld_deInit(gI2cLldHandle[MCU_I2C0_CH0]);
               return ti_status;
            }

            /* Set I2C configuration flag to indicate successful setup */
            gUARTCmdObjects.uarti2cparam.uI2CHandle = 1;

            /* Log the configured bit rate for debugging */
            if(gI2cLldObjects[MCU_I2C0_CH0].bitRate)
            {
                DebugP_logInfo("[MCU_I2C0] Bit Rate = %dKHz\r\n", 400);
            }
            else
            {
                DebugP_logInfo("[MCU_I2C0] Bit Rate = %dKHz\r\n", 100);
            }
            break;
        }

        case SOC_I2C0_CH1:
        {
            /* Close any previously opened driver */
            I2C_lld_deInit(gI2cLldHandle[SOC_I2C0_CH1]);   
            /* Clear user configuration flag */
            gUARTCmdObjects.uarti2cparam.uI2CHandle = 0;
            
            /* Update SOC I2C configuration with new speed */
            gI2cLldObjects[SOC_I2C0_CH1].bitRate = speed;
              
            /* Initialize and open I2C driver */
            gI2cLldHandle[SOC_I2C0_CH1] = &gI2cLldObjects[SOC_I2C0_CH1];
            ti_status += I2C_lld_init(gI2cLldHandle[SOC_I2C0_CH1]);

            if (ti_status != SystemP_SUCCESS)
            {
               /* Close driver if initialization fails */
               I2C_lld_deInit(gI2cLldHandle[SOC_I2C0_CH1]);
               return SystemP_FAILURE;
            }

            /* Set I2C configuration flag */
            gUARTCmdObjects.uarti2cparam.uI2CHandle = 1;
            
            /* Log the configured bit rate for debugging */
            if(gI2cLldObjects[SOC_I2C0_CH1].bitRate)
            {
                DebugP_logInfo("[MAIN_I2C0] Bit Rate = %dKHz\r\n", 400);
            }
            else
            {
                DebugP_logInfo("[MAIN_I2C0] Bit Rate = %dKHz\r\n", 100);
            }
            break;
        }
    }
    return SystemP_SUCCESS;
}

/**
 * @brief Configure I2C device address based on address length
 * @param i2cAdd I2C slave address
 * @param param Pointer to I2C extended parameters structure
 */
void I2C_DEVICE_ADDRESS_SET(uint16_t i2cAdd, uint8_t regSize, I2C_ExtendedParams* param)
{
    if (param == NULL)
    {
        DebugP_logError("ERROR: NULL pointer passed to I2C_DEVICE_ADDRESS_SET\r\n");
        return;
    }

    if (gUARTCmdObjects.uarti2cparam.uI2CHandle)
    {
        /* Configure 7-bit I2C address */
        if (gUARTCmdObjects.uarti2cparam.uI2CAddLen == SOC_I2C0_ADDRESS_LEN_7BIT)
        {
            param->expandSA = FALSE;
            param->deviceAddress = (BYTE)(i2cAdd & 0x7F);
        }
        /* Configure 10-bit I2C address */
        else if (gUARTCmdObjects.uarti2cparam.uI2CAddLen == SOC_I2C0_ADDRESS_LEN_10BIT)
        {
            param->expandSA = TRUE;
            param->deviceAddress = i2cAdd;
        }
        else
        {
            DebugP_logError("ERROR: Invalid I2C address length (%d)\r\n", gUARTCmdObjects.uarti2cparam.uI2CAddLen);
        }

        /* Set the I2C internal register address size */
        gUARTCmdObjects.uarti2cparam.uI2CRegSize = regSize;

        DebugP_logInfo("I2C SlaveAddr is 0X%X (using %d bits) and RegAddrSize is %d Bytes.\r\n", \
            param->deviceAddress, (gUARTCmdObjects.uarti2cparam.uI2CAddLen == 0) ? 7:10, regSize \
        );

    } 
    else
    {
        DebugP_logError("ERROR: Please open I2C driver\n");
    }
}

/**
 * @brief Initialize I2C transaction buffer with register address
 * @param paramT Pointer to I2C extended parameters structure
 * @param regAdd Register address to write to buffer
 * @param dataSize Size of register address (8, 16, 24, or 32 bits)
 */
void Init_I2C_Transaction_Buf(I2C_ExtendedParams* paramT, regAddress regAdd, BYTE dataSize)
{
    if (paramT == NULL)
    {
        DebugP_logError("ERROR: NULL pointer passed to Init_I2C_Transaction_Buf\r\n");
        return;
    }
    if(paramT->buffer == NULL)
    {
        DebugP_logError("ERROR: NULL buffer passed to Init_I2C_Transaction_Buf\r\n");
        return;
    }

    switch (dataSize)
    {
        case I2C_8BIT_DATASIZE:
            paramT->buffer[0] = regAdd & 0xFF;
            break;

        case I2C_16BIT_DATASIZE:
            paramT->buffer[0] = (regAdd >> 8) & 0xFF;  /* MSB */
            paramT->buffer[1] = regAdd & 0xFF;         /* LSB */
            break;

        case I2C_24BIT_DATASIZE:
            paramT->buffer[0] = (regAdd >> 16) & 0xFF;
            paramT->buffer[1] = (regAdd >> 8) & 0xFF;
            paramT->buffer[2] = regAdd & 0xFF;
            break;

        case I2C_32BIT_DATASIZE:
            paramT->buffer[0] = (regAdd >> 24) & 0xFF;
            paramT->buffer[1] = (regAdd >> 16) & 0xFF;
            paramT->buffer[2] = (regAdd >> 8) & 0xFF;
            paramT->buffer[3] = regAdd & 0xFF;
            break;

        default:
            DebugP_logError("ERROR: Unsupported I2C data size: %d\r\n", dataSize);
            break;
    }
}

/**
 * @brief Write a single byte to an I2C device register
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param val Pointer to value to write
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_RegWrite(UART_OBJ* uI2CConfigParam, BYTE* val)
{
    ti_status = SystemP_SUCCESS;
    I2C_ExtendedParams regWritePara = {0};
    uint8_t buffer[5];
    regWritePara.buffer = buffer;

    if (uI2CConfigParam == NULL) 
    {
        DebugP_logError("ERROR: I2C config parameter is not valid\r\n");
        return -1;
    }

    if (!uI2CConfigParam->uarti2cparam.uI2CHandle) 
    {
        DebugP_logError("ERROR: I2C driver is not open for channel %d\r\n", uI2CConfigParam->uarti2cparam.uI2CChannel);
        return -1;
    }

    /* Set device address for I2C communication */
    I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &regWritePara);

    /* Initialize transaction buffer with register address */
    Init_I2C_Transaction_Buf(&regWritePara, uI2CConfigParam->uarti2cparam.uI2CRegAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize);
    regWritePara.buffer[uI2CConfigParam->uarti2cparam.uI2CRegSize] = *val;  /* Value to write */
    regWritePara.size = 1 + uI2CConfigParam->uarti2cparam.uI2CRegSize;

    ti_status += I2C_lld_write(gI2cLldHandle[uI2CConfigParam->uarti2cparam.uI2CChannel], &regWritePara, I2C_TIMEOUT_uS);


    uint8_t readByte = 0;
    regWritePara.buffer = &readByte;
    regWritePara.size = 1;

    I2C_lld_read(gI2cLldHandle[uI2CConfigParam->uarti2cparam.uI2CChannel], &regWritePara, I2C_TIMEOUT_uS);
    
    if (ti_status != I2C_STS_SUCCESS) 
    {
        DebugP_logError("ERROR: Register write failed on channel %d\r\n", uI2CConfigParam->uarti2cparam.uI2CChannel);
        return ti_status;
    }

    return I2C_STS_SUCCESS;
}


/**
 * @brief Read a single byte from an I2C device register
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param readByte Pointer to store the read byte
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_RegRead(UART_OBJ* uI2CConfigParam, BYTE* readByte)
{
    ti_status = SystemP_SUCCESS;
    BYTE writeAdd[5];

    const char* channel_str = (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) ? "MCU_I2C0" :
                          (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) ? "MAIN_I2C0" : "UNKNOWN";

    I2C_ExtendedParams regWrite = {0}, readRawPar = {0};
    regWrite.buffer = writeAdd;
   
    if (!uI2CConfigParam->uarti2cparam.uI2CHandle)
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\r\n", channel_str);
        return -1;
    }

    /* Set device address for write operation */
    I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &regWrite);
    
    /* Write register address to device */
    Init_I2C_Transaction_Buf(&regWrite, uI2CConfigParam->uarti2cparam.uI2CRegAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize);
    regWrite.size = uI2CConfigParam->uarti2cparam.uI2CRegSize;

    ti_status += I2C_lld_write(gI2cLldHandle[uI2CConfigParam->uarti2cparam.uI2CChannel], &regWrite, I2C_TIMEOUT_uS);
    if (ti_status != I2C_STS_SUCCESS)
    {
        DebugP_logError("[%s] : Write Operation Fail\r\n", channel_str);
        return ti_status;
    }

    /* Read register value from device */
    I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &readRawPar);
    readRawPar.buffer = readByte;
    readRawPar.size = 1;

    ti_status += I2C_lld_read(gI2cLldHandle[uI2CConfigParam->uarti2cparam.uI2CChannel], &readRawPar, I2C_TIMEOUT_uS);
    if (ti_status != I2C_STS_SUCCESS)
    {
        DebugP_logError("ERROR: [%s] Read operation failed\r\n", channel_str);
        return ti_status;
    }

    return I2C_STS_SUCCESS;
}
/**
 * @brief Write data to multiple registers of an I2C device
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param start_regAdd Starting register address
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer to write
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_MultiRegWrite(UART_OBJ* uI2CConfigParam, regAddress start_regAdd, BYTE nBytes, BYTE* data)
{
    ti_status = SystemP_SUCCESS;
    I2C_ExtendedParams wrt_Config = {0};
    BYTE tempBytes = nBytes;
    int i2c_channel_index = -1;
    uint8_t temp111[256];
    
    wrt_Config.buffer = temp111;
    
    const char* channel_str = (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) ? "MCU_I2C0" :
                          (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) ? "MAIN_I2C0" : "UNKNOWN";

    /* Set the appropriate I2C channel index based on channel selection */
    switch (uI2CConfigParam->uarti2cparam.uI2CChannel) 
    {
        case MCU_I2C0_CH0:
            i2c_channel_index = MCU_I2C0_CH0;
            break;
        case SOC_I2C0_CH1:
            i2c_channel_index = SOC_I2C0_CH1;
            break;
        default:
            DebugP_logError("Invalid I2C Channel\r\n");
            return -1;
    }

    if (i2c_channel_index != -1 && uI2CConfigParam->uarti2cparam.uI2CHandle) 
    {
        /* Set the I2C address and configure the I2C device */
        I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &wrt_Config);
        /* Initialize I2C transaction buffer with register address */
        Init_I2C_Transaction_Buf(&wrt_Config, start_regAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize);
        /* Populate the buffer with the data to be written */
        for (BYTE i = 0; i < tempBytes; i++) 
        {
            wrt_Config.buffer[i + uI2CConfigParam->uarti2cparam.uI2CRegSize] = data[i];
        }
        /* Set the size of the transaction */
        wrt_Config.size = tempBytes + uI2CConfigParam->uarti2cparam.uI2CRegSize;
        /* Perform the I2C write operation */
        ti_status += I2C_lld_write(gI2cLldHandle[i2c_channel_index], &wrt_Config, I2C_TIMEOUT_uS);
        if (ti_status != I2C_STS_SUCCESS) 
        {
            DebugP_logError("[%s] : Write Operation Fail\r\n", channel_str);
            return ti_status;
        }
    } 
    else 
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\r\n", channel_str);
        return -1;
    }

    return I2C_STS_SUCCESS;
}

/**
 * @brief Read data from multiple registers of an I2C device
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param start_regAdd Starting register address
 * @param nBytes Number of bytes to read
 * @param data Pointer to buffer to store read data
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_MultiRegRead(UART_OBJ* uI2CConfigParam, regAddress start_regAdd, BYTE nBytes, BYTE *data)
{
    ti_status = SystemP_SUCCESS;
    I2C_ExtendedParams wrt_Config, rd_Config;
    int i2c_channel_index = -1;

    const char* channel_str = (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) ? "MCU_I2C0" :
                          (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) ? "MAIN_I2C_CH0" : "UNKNOWN";

    /* Set the appropriate I2C channel index based on channel selection */
    switch (uI2CConfigParam->uarti2cparam.uI2CChannel) 
    {
        case MCU_I2C0_CH0:
            i2c_channel_index = MCU_I2C0_CH0;
            break;
        case SOC_I2C0_CH1:
            i2c_channel_index = SOC_I2C0_CH1;
            break;
        default:
            DebugP_logError("Invalid I2C Channel\r\n");
            return -1;
    }

    if (i2c_channel_index != -1 && uI2CConfigParam->uarti2cparam.uI2CHandle) 
    {
        /* Set the register address to be written */
        I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &wrt_Config);
        /* Initialize I2C transaction buffer with register address */
        Init_I2C_Transaction_Buf(&wrt_Config, start_regAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize);
        wrt_Config.size = uI2CConfigParam->uarti2cparam.uI2CRegSize; /* Only register write */
        /* Perform the write operation for register address */
        ti_status += I2C_lld_write(gI2cLldHandle[i2c_channel_index], &wrt_Config, I2C_TIMEOUT_uS);
        if (ti_status != I2C_STS_SUCCESS) 
        {
            DebugP_logError("ERROR: [%s] Multi-register address write failed\r\n", channel_str);
            return ti_status;
        }

        /* Read the register values */
        I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &rd_Config);
        rd_Config.buffer = data;
        rd_Config.size = nBytes;

        /* Perform the read operation for register values */
        ti_status += I2C_lld_read(gI2cLldHandle[i2c_channel_index], &rd_Config, I2C_TIMEOUT_uS);
        if (ti_status != I2C_STS_SUCCESS) 
        {
            DebugP_logError("ERROR: [%s] Multi-register read failed\r\n", channel_str);
            return ti_status;
        }
    } 
    else 
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\r\n", channel_str);
        return -1;
    }

    return I2C_STS_SUCCESS;
}


/**
 * @brief Write raw data to an I2C device
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer to write
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_RawWrite(UART_OBJ* uI2CConfigParam, BYTE nBytes, BYTE *data)
{
    ti_status = SystemP_SUCCESS;
    I2C_ExtendedParams rawWriteConfig;
    uint8_t txBuff[256];
    BYTE tempBytes = nBytes;
    int i2c_channel_index = -1;

    /* Define channel string for debug logs */
    const char* channel_str = (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) ? "MCU_I2C0" :
                          (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) ? "MAIN_I2C0" : "UNKNOWN";

    /* Define channel-specific settings */
    if (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) 
    {
        i2c_channel_index = MCU_I2C0_CH0;
    } 
    else if (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) 
    {
        i2c_channel_index = SOC_I2C0_CH1;
    } 
    else 
    {
        DebugP_logError("Invalid I2C Channel Selection\r\n");
        return -1;
    }
    /* Set buffer for raw write */
    rawWriteConfig.buffer = txBuff;

    /* Check if the driver is open */
    if (i2c_channel_index != -1 && uI2CConfigParam->uarti2cparam.uI2CHandle) 
    {
        /* Set I2C address */
        I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &rawWriteConfig);

        /* Copy data to buffer */
        for (BYTE i = 0; i < tempBytes; i++) 
        {
            rawWriteConfig.buffer[i] = data[i];
        }

        /* Set transaction size */
        rawWriteConfig.size = tempBytes;

        /* Write data to I2C */
        ti_status += I2C_lld_write(gI2cLldHandle[i2c_channel_index], &rawWriteConfig, I2C_TIMEOUT_uS);
        if (ti_status != I2C_STS_SUCCESS) 
        {
            DebugP_logError("ERROR: [%s] Raw write operation failed\r\n", channel_str);
            return ti_status;
        }
    } 
    else 
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\r\n", channel_str);
        return -1;
    }

    return I2C_STS_SUCCESS;
}

/**
 * @brief Read raw data from an I2C device
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param nBytes Number of bytes to read
 * @param data Pointer to buffer to store read data
 * @return I2C_STS_SUCCESS on success, error code on failure
 */
int I2C_RawRead(UART_OBJ* uI2CConfigParam, BYTE nBytes, BYTE *data)
{
    ti_status = SystemP_SUCCESS;
    I2C_ExtendedParams rawReadConfigParam;
    int i2c_channel_index = -1;

    /* Define channel string for debug logs */
    const char* channel_str = (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) ? "I2C_CH0" :
                          (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) ? "I2C_CH1" : "UNKNOWN";

    /* Determine the correct channel index based on the selected channel */
    if (uI2CConfigParam->uarti2cparam.uI2CChannel == MCU_I2C0_CH0) 
    {
        i2c_channel_index = MCU_I2C0_CH0;
    } 
    else if (uI2CConfigParam->uarti2cparam.uI2CChannel == SOC_I2C0_CH1) 
    {
        i2c_channel_index = SOC_I2C0_CH1;
    } 
    else 
    {
        DebugP_logError("Invalid I2C Channel Selection\r\n");
        return -1;
    }

    /* Check if the driver is open */
    if (i2c_channel_index != -1 && uI2CConfigParam->uarti2cparam.uI2CHandle) 
    {
        /* Set I2C address and configure read parameters */
        I2C_DEVICE_ADDRESS_SET(uI2CConfigParam->uarti2cparam.uI2CSlaveAdd, uI2CConfigParam->uarti2cparam.uI2CRegSize, &rawReadConfigParam);

        /* Set read buffer and size */
        rawReadConfigParam.buffer = data;
        rawReadConfigParam.size = nBytes;

        /* Perform the read operation */
        ti_status += I2C_lld_read(gI2cLldHandle[i2c_channel_index], &rawReadConfigParam, I2C_TIMEOUT_uS);
        if (ti_status != I2C_STS_SUCCESS) 
        {
            DebugP_logError("ERROR: [%s] Raw read operation failed\r\n", channel_str);
            return ti_status;
        }
    } 
    else 
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\r\n", channel_str);
        return -1;
    }

    return I2C_STS_SUCCESS;
}

/**
 * @brief Perform block write followed by block read operation
 * @param uI2CConfigParam Pointer to UART I2C configuration parameters
 * @param nWriteBytes Number of bytes to write
 * @param writeData Pointer to data to write
 * @param nReadBytes Number of bytes to read
 * @param readData Pointer to buffer to store read data
 * @return SystemP_SUCCESS on success, error code on failure
 */
int I2C_BlockWriteRead(UART_OBJ* uI2CConfigParam, BYTE nWriteBytes, BYTE *writeData, BYTE nReadBytes, BYTE* readData)
{
    ti_status = SystemP_SUCCESS;

    /* Write data to I2C device */
    ti_status += I2C_RawWrite(uI2CConfigParam, nWriteBytes, writeData);
    if (ti_status != SystemP_SUCCESS)
    {
        DebugP_logError("ERROR: Write operation failed\r\n");
        return ti_status;
    }

    /* Read data from I2C device */
    ti_status += I2C_RawRead(uI2CConfigParam, nReadBytes, readData);
    if (ti_status != SystemP_SUCCESS)
    {
        DebugP_logError("ERROR: Read operation failed\r\n");
        return ti_status;
    }

    return SystemP_SUCCESS;
}
