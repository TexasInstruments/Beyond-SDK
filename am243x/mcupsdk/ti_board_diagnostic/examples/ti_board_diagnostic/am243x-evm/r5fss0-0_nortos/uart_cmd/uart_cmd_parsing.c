/**
 * @file uart_cmd_parsing.c
 * @brief UART command parsing and processing for TI AM2432 Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements UART command parsing and processing functionality for
 * various communication protocols including I2C, SPI, UART, GPIO, PWM, DAC,
 * CAN FD, and interrupt handling. Provides command interface for hardware
 * control and data acquisition via UART COM PORT.
 */

#include "../header_files/common_extern.h"
#include "../header_files/uart_cmd_header.h"
#include <stdint.h>

#define MAX_CMD_DATA_LEN            256

/* UART protocol signature characters (hex values for 'T' and 'I') */
#define SIGN_T_CHAR_HEX                   0x54 // 'T'
#define SIGN_I_CHAR_HEX                   0x49 // 'I'

/* UART command header field positions */
#define UART_CMD_FUN_CODE_ID              6U
#define UART_CMD_CODE_ID            UART_CMD_FUN_CODE_ID + 1
#define UART_DATA_LEN_ID            UART_CMD_CODE_ID + 1

/* Return status codes */
#define RETURN_FALSE -1;
#define RETURN_TRUE   1;

/* Global UART object and parsing state */
extern UART_OBJ gUARTCmdObjects;


/* Status and data buffer variables */
int32_t ti_status = SystemP_SUCCESS;
int32_t TI_StatusCode = SystemP_SUCCESS;
uint8_t dataBuf[MAX_CMD_DATA_LEN] = {0}; 
uint8_t dacInitState = 0;

/* Function code enumeration */
enum TI_FUNCTION_CODE gFun_Code;


/**
 * @brief Write UART command response header
 * @param uartParm Pointer to UART object containing header information
 * @param cmdCode Command response code
 * @param dataLen Length of data payload
 * @param statusCode Operation status code
 * 
 * Constructs and sends a 10-byte UART command response header containing
 * signature, sequence number, function code, command code, data length,
 * and status code.
 */
static void UARTWriteCMD(UART_OBJ* uartParm, uint8_t cmdCode, uint8_t dataLen, uint8_t statusCode)
{
    uint8_t writeBufUART[10];

    /* SIGNATURE */
    writeBufUART[0] = 0x54; // 'T'
    writeBufUART[1] = 0x49; // 'I'
    /* SEQUENCE NUMBER - 2BYTES */
    writeBufUART[2] = uartParm->headBuf[2];
    writeBufUART[3] = uartParm->headBuf[3];
    /* RESERVED BYTES - 2BYTES */
    writeBufUART[4] = 0;
    writeBufUART[5] = 0;
    /* FUNCTION CODE - 1BYTE */
    writeBufUART[6] = uartParm->headBuf[6];
    /* COMMAND CODE - 1BYTE */
    writeBufUART[7] = cmdCode;
    /* DATA LEN - 1BYTE */
    writeBufUART[8] = dataLen;
    /* STATUS CODE - 1BYTE */
    writeBufUART[9] = statusCode;

    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf = writeBufUART;
    trans.count = 10;
    if(UART_write(gUartHandle[CONFIG_UART1], &trans) != SystemP_SUCCESS)
    {
        DebugP_logError("UART WRITE OPERATION FAIL..\n");
    }
}

/**
 * @brief Write raw data buffer to UART
 * @param uartBuf Pointer to data buffer to send
 * @param bufLen Length of data buffer
 * 
 * Sends raw data payload to UART CDC interface with error handling.
 */
static void UARTWriteData(void* uartBuf, size_t bufLen)
{
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf = uartBuf;
    trans.count = bufLen;
    if(UART_write(gUartHandle[CONFIG_UART1], &trans))
    {
        DebugP_logError("UART WRITE OPERATION FAIL..\n");
    }
}


/**
 * @brief Send interrupt packet information via UART
 * @param channel Interrupt channel number
 * @param counts Interrupt event count
 * 
 * Sends a 5-byte packet containing channel number and 32-bit
 * interrupt count (big-endian format) to UART interface.
 */
void SendISRPackets(uint8_t ch, uint32_t counts)
{
    uint8_t sendISRBuff[5];
    sendISRBuff[0] = ch;
    /* Convert 32-bit count to big-endian bytes */
    sendISRBuff[1] = (counts >> 24) & 0xFF;
    sendISRBuff[2] = (counts >> 16) & 0xFF;
    sendISRBuff[3] = (counts >> 8) & 0xFF;
    sendISRBuff[4] = counts & 0xFF;
    UARTWriteData(sendISRBuff,5);
}

/**
 * @brief Initialize I2C parameters structure
 * @param uartParm Pointer to UART object containing I2C parameters
 * 
 * Resets all I2C parameters to default values (zero).
 */
static void uI2CStructInit(UART_OBJ* uartParm)
{
    uartParm->uarti2cparam.uI2CChannel = 0;
    uartParm->uarti2cparam.uI2CAddLen = 0;
    uartParm->uarti2cparam.uI2CDatabufLen = 0;
    uartParm->uarti2cparam.uI2CPullState = 0;
    uartParm->uarti2cparam.uI2CRegAdd = 0;
    uartParm->uarti2cparam.uI2CSlaveAdd = 0;
    uartParm->uarti2cparam.uI2CSpeed = 0;
    DebugP_logWarn("I2C_struct initialized to zero...\r\n");
}

/**
 * @brief Initialize SPI parameters structure
 * @param uartParm Pointer to UART object containing SPI parameters
 * 
 * Resets all SPI parameters to default values (zero).
 */
static void uSPIStructInit(UART_OBJ* uartParm)
{
    uartParm->uartspiparam.uSPI_CSType = 0;
    uartParm->uartspiparam.uSPI_Speed = 0;
    uartParm->uartspiparam.uSPIChannel = 0;
    uartParm->uartspiparam.uSPIHandle = 0;
}

/**
 * @brief Set I2C channel and slave address from data buffer
 * 
 * Extracts I2C channel and slave address from global dataBuf.
 * Slave address is reconstructed from MSB and LSB bytes.
 */
static void Set_I2C_Channel_and_SlaveAdd(uint8_t cmdCode)
{
    gUARTCmdObjects.uarti2cparam.uI2CChannel = dataBuf[0]; /* I2C Channel */
    gUARTCmdObjects.uarti2cparam.uI2CSlaveAdd = dataBuf[2]; /* SLAVE ADD - LSB */
    gUARTCmdObjects.uarti2cparam.uI2CSlaveAdd |= (((uint16_t)dataBuf[1]) << 8); /* SLAVE ADD - MSB */
    if(cmdCode <= I2C_MULTIREG_READ_CMD_CODE)
    {
        // Not required for RAW & Block read/Write
        gUARTCmdObjects.uarti2cparam.uI2CRegSize = dataBuf[3];
        uint16_t regAddr = (dataBuf[3] == 1) ? dataBuf[4] : ((dataBuf[4] << 8) | dataBuf[5]);
        gUARTCmdObjects.uarti2cparam.uI2CRegAdd = regAddr;
    }
    else
    {
        gUARTCmdObjects.uarti2cparam.uI2CRegSize = 0;
    }
    
}

static int8_t I2CcmdCheck(enum TI_I2C_COMMAND_CODE i2cCommandState)
{
    switch (i2cCommandState) {
        case I2C_CONFIG_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Initialize I2C Structure */
            uI2CStructInit(&gUARTCmdObjects);
            /* Extract I2C configuration parameters from data buffer */
            gUARTCmdObjects.uarti2cparam.uI2CChannel = dataBuf[0]; /* I2C Channel */
            gUARTCmdObjects.uarti2cparam.uI2CSpeed =  dataBuf[1]; /* I2C Speed */
            gUARTCmdObjects.uarti2cparam.uI2CAddLen =  dataBuf[2]; /* I2C Address Length */

            TI_StatusCode = I2C_Config(gUARTCmdObjects.uarti2cparam.uI2CChannel,gUARTCmdObjects.uarti2cparam.uI2CSpeed);
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("[UART_ERROR]: I2C DRIVER OPEN FAIL \n");
            }
            
            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,0x01,TI_StatusCode);
            break;
        }

        case I2C_REG_WRITE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Extract I2C channel and slave address from data buffer */
            Set_I2C_Channel_and_SlaveAdd(I2C_REG_WRITE_CMD_CODE);

            /* Write single byte to specified register */
            TI_StatusCode += I2C_RegWrite(&gUARTCmdObjects,&dataBuf[4+dataBuf[3]]) ;
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("I2C register write failed\n");
            }

            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x81,0x01,TI_StatusCode);
            break;
        }

        case I2C_REG_READ_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            uint8_t regReadValue[1] = {0x00};

            /* Extract I2C channel and slave address from data buffer */
            Set_I2C_Channel_and_SlaveAdd(I2C_REG_READ_CMD_CODE);

            /* Read single byte from specified register */
            TI_StatusCode += I2C_RegRead(&gUARTCmdObjects,&regReadValue[0]);

            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("I2C register read failed\n");
            }

            /* Send response header and data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x82,0x02,TI_StatusCode); /* Header */
            UARTWriteData(regReadValue,1); /* Data */
            break;
        }

        case I2C_MULTIREG_WRITE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Extract I2C channel and slave address from data buffer */
            Set_I2C_Channel_and_SlaveAdd(I2C_MULTIREG_WRITE_CMD_CODE);
           
            BYTE tBytes = dataBuf[4+dataBuf[3]]; /* Total bytes to write */
            /* Write multiple bytes starting from specified register */
            TI_StatusCode += I2C_MultiRegWrite(&gUARTCmdObjects,gUARTCmdObjects.uarti2cparam.uI2CRegAdd,tBytes, &dataBuf[5+dataBuf[3]]);
            if(TI_StatusCode !=I2C_STS_SUCCESS)
            {
                DebugP_logError("I2C Multi-register write failed\n");
            }
           
            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x83,0x01,TI_StatusCode); /* Header */
            break;
        }

        case I2C_MULTIREG_READ_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Extract I2C channel and slave address from data buffer */
            Set_I2C_Channel_and_SlaveAdd(I2C_MULTIREG_READ_CMD_CODE);

            BYTE tBytes = dataBuf[4+dataBuf[3]]; /* Total bytes to read */

            /* Allocate buffer for multiple register read */
            BYTE* multiRegReadBuff = (BYTE*) calloc(tBytes, sizeof(BYTE));
            if(multiRegReadBuff == NULL)
            {
                DebugP_logError("Failed to allocate memory\n");
            }

            /* Read multiple bytes starting from specified register */
            TI_StatusCode += I2C_MultiRegRead(&gUARTCmdObjects,gUARTCmdObjects.uarti2cparam.uI2CRegAdd,tBytes,multiRegReadBuff);
            if(TI_StatusCode !=I2C_STS_SUCCESS)
            {
                DebugP_logError("I2C Multi-register read failed\n");               
            }
            /* Send response header and data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x84,tBytes+1,TI_StatusCode); /* Header */
            UARTWriteData(multiRegReadBuff,tBytes); /* Data */

            /* Free allocated memory */
            free(multiRegReadBuff);
            break;
        }

        default:
            return RETURN_FALSE;       
            break;
    }
    return RETURN_TRUE;
}

static int8_t SPIcmdCheck(enum TI_SPI_COMMAND_CODE spiCommandState)
{
    switch (spiCommandState) {
        case SPI_CONFIG_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Initialize SPI Structure */
            uSPIStructInit(&gUARTCmdObjects);
            /* Extract SPI configuration parameters from data buffer */
            gUARTCmdObjects.uartspiparam.uSPIChannel = dataBuf[0]; /* SPI Channel */
            gUARTCmdObjects.uartspiparam.uSPIMode =  dataBuf[2]; /* SPI mode */
            /* Configure chip select type: 0=manual, 1=auto */
            if(!dataBuf[3])gUARTCmdObjects.uartspiparam.uSPI_CSType = TRUE;
            else if (dataBuf[3] == 1)gUARTCmdObjects.uartspiparam.uSPI_CSType = FALSE;
        
            gUARTCmdObjects.uartspiparam.uSPI_CS_Polarity =  dataBuf[4]; /* SPI CS Polarity */
            /* Reconstruct 32-bit SPI speed from 4 bytes (big-endian) */
            gUARTCmdObjects.uartspiparam.uSPI_Speed = dataBuf[8];
            gUARTCmdObjects.uartspiparam.uSPI_Speed |= (dataBuf[7] << 8);
            gUARTCmdObjects.uartspiparam.uSPI_Speed |= (dataBuf[6] << 16);
            gUARTCmdObjects.uartspiparam.uSPI_Speed |= (dataBuf[5] << 24);
            
            /* Configure SPI with extracted parameters */
            TI_StatusCode += SPI_Config(gUARTCmdObjects.uartspiparam.uSPIChannel,dataBuf[1],gUARTCmdObjects.uartspiparam.uSPIMode,gUARTCmdObjects.uartspiparam.uSPI_Speed,
                    gUARTCmdObjects.uartspiparam.uSPI_CSType,gUARTCmdObjects.uartspiparam.uSPI_CS_Polarity);
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("[UART_ERROR]: SPI DRIVER OPEN FAIL \n");
            }
            
            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,0x01,TI_StatusCode);
            break;
        }

        case SPI_READWRITE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            gUARTCmdObjects.uartspiparam.uSPIChannel = dataBuf[0]; /* SPI Channel */

            /* Allocate buffer for SPI receive data */
            uint8_t* rxSPIBuffData = (uint8_t*) calloc(dataBuf[1],sizeof(uint8_t));
            if(rxSPIBuffData == NULL)
            {
                DebugP_logError("Dynamic memory allocation failed\n");
            }

            /* Perform simultaneous SPI read and write operation */
            TI_StatusCode += SPI_ReadWrite(&gUARTCmdObjects,&dataBuf[2],rxSPIBuffData,dataBuf[1]);
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("[UART_ERROR]: SPI READ-WRITE FAIL \n");
            }

            /* Send response header and data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x81,(dataBuf[1]+1),TI_StatusCode);
            UARTWriteData(rxSPIBuffData,dataBuf[1]);

            /* Free allocated memory */
            free(rxSPIBuffData);
            break;
        }
        default:
            return RETURN_FALSE;       
            break;
    }
    return RETURN_TRUE;
}

/**
 * @brief Process UART commands and execute corresponding operations
 * @param uartCommandState UART command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles UART operations including configuration, read/write, receiver control,
 * and mode setting. Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  UARTcmdCheck(enum TI_UART_COMMAND_CODE uartCommandState)
{
    switch (uartCommandState) 
    {

        case UART_CONFIG_CMD_CODE:
        {
            TI_StatusCode = 0;

            gUARTCmdObjects.uartuartparam.uUARTChannel = dataBuf[0]; /* UART Channel */

            /* Configure UART with baud rate, data bits, stop bits, and parity */
            TI_StatusCode +=  UART_CONFIG(gUARTCmdObjects.uartuartparam.uUARTChannel,dataBuf[1],
                    dataBuf[2],dataBuf[3],dataBuf[4]);

            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("UART configuration failed\n");
            }

            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,0x01,TI_StatusCode);
            break;
        }

        case UART_WRITE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Write data to UART */
            TI_StatusCode += UART_Write(&gUARTCmdObjects,dataBuf[1],&dataBuf[2]);
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("UART write operation failed\n");
            }

            /* Send response data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x81,0x01,TI_StatusCode);
            break;
        }

        case UART_READ_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            
            /* Allocate buffer for UART read data */
            uint8_t *uartReadBuff = (uint8_t*) calloc(dataBuf[1],sizeof(uint8_t));
            if(uartReadBuff == NULL)
            {
                DebugP_logError("Dynamic memory allocation failed\n");
            }

            /* Read data from UART */
            TI_StatusCode = UART_Read(&gUARTCmdObjects,dataBuf[1],uartReadBuff);
            if(TI_StatusCode != SystemP_SUCCESS)
            {
                DebugP_logError("UART read operation failed\n");
            }

            /* Send response header and data on UART */
            UARTWriteCMD(&gUARTCmdObjects,0x82,(dataBuf[1]+1),TI_StatusCode);
            UARTWriteData(uartReadBuff,dataBuf[1]);
            /* Free allocated memory */
            free(uartReadBuff);
            break;
        }

        default:
            return RETURN_FALSE;

    
    }
    return RETURN_TRUE;
}

/**
 * @brief Process GPIO commands and execute corresponding operations
 * @param gpioCommandState GPIO command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles GPIO operations including configuration, state control, port operations,
 * and pulse generation. Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  GPIOcmdCheck(enum TI_GPIO_COMMAND_CODE gpioCommandState)
{
    switch (gpioCommandState) 
    {
        case GPIO_CONFIG_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Configure GPIO pin direction and control */
            TI_StatusCode = GPIO_WriteControl(&dataBuf[0]);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,1,TI_StatusCode);
            break;
        }
        case GPIO_WRITE_STATE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Write GPIO pin state (high/low) */
            TI_StatusCode = GPIO_WriteState(&dataBuf[0]);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x81,1,TI_StatusCode);
            break;
        }
        case GPIO_READ_STATE_CMD_CODE:
        {
            uint8_t readGPIOState[8];
            uint8_t nBytesGPIO;

            /* Read GPIO state for multiple pins */
            nBytesGPIO = GPIO_ReadState(dataBuf[0],readGPIOState);
            UARTWriteCMD(&gUARTCmdObjects,0x82,(1+nBytesGPIO),TI_StatusCode);
            /* Send status code to UART */
            UARTWriteData(readGPIOState,nBytesGPIO);
            break;
        }

        case GPIO_SET_PORT_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Set entire GPIO port value */
            TI_StatusCode = GPIO_Set(dataBuf[0],dataBuf[1]);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x83,1,TI_StatusCode);
            break;
        }

        case GPIO_WRITE_PORT_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Write entire GPIO port value */
            TI_StatusCode = GPIO_Write(dataBuf[0],dataBuf[1]);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x84,1,TI_StatusCode);
            break;       
        }

        case GPIO_READ_PORT_CMD_CODE:
        {
            uint8_t readPinState;
            /* Read single GPIO pin state */
            readPinState = GPIO_Read(dataBuf[0]);
            /*
                GPIO_READ_ERROR = -1,  Error or invalid pin 
                GPIO_READ_LOW   = 0,    Pin is LOW 
                GPIO_READ_HIGH  = 1     Pin is HIGH 
            */
            UARTWriteCMD(&gUARTCmdObjects,0x85,1,readPinState);
            break;
        }
        case GPIO_WRITE_PULSE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Reconstruct 16-bit pulse time from 2 bytes */
            uint16_t pulseTime = dataBuf[3];
            pulseTime |= (dataBuf[2] << 8);
            /* Generate pulse on GPIO pin with specified duration */
            TI_StatusCode = GPIO_WritePulse(dataBuf[0],dataBuf[1],pulseTime);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x86,1,TI_StatusCode);
            break;
        }
        default:
            return RETURN_FALSE;  
    
    }
    return RETURN_TRUE;
}

/**
 * @brief Process interrupt commands and execute corresponding operations
 * @param intCommandState Interrupt command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles interrupt operations including configuration, event counting,
 * and callback handler management. Processes data from the global dataBuf
 * and sends responses via UART.
 */
static int8_t  INTcmdCheck(enum TI_INT_COMMAND_CODE intCommandState)
{
    switch (intCommandState) 
    {

        case INT_CONFIG_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;

            /* Configure interrupt trigger types for all three interrupt channels */
            TI_StatusCode += interruptConfig(0,(TriggerType)dataBuf[0]); /* INT0 */
            TI_StatusCode += interruptConfig(1,(TriggerType)dataBuf[1]); /* INT1 */
            TI_StatusCode += interruptConfig(2,(TriggerType)dataBuf[2]); /* INT2 */

            /* Disable all interrupt callbacks initially */
            intCallBack[INT0].intCallBackSet = FALSE;
            intCallBack[INT1].intCallBackSet = FALSE;
            intCallBack[INT2].intCallBackSet = FALSE;
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,1,TI_StatusCode);
            break;
        }

        case INT_CHECK_RECEIVED_CMD_CODE:
        {
            int32_t intEventCount[1];
            TI_StatusCode = SystemP_SUCCESS;

            switch(dataBuf[0])
            {
                case INT0: 
                {
                    /* Store interrupt event count for INT0 */
                    intEventCount[0] = _INT_EventCout[INT0];
                    if(dataBuf[1]) _INT_EventCout[INT0] = 0; /* Reset Event Counter */
                    break;
                }
                case INT1:
                {   
                    /* Store interrupt event count for INT1 */
                    intEventCount[0] = _INT_EventCout[INT1];
                    if(dataBuf[1]) _INT_EventCout[INT1] = 0; /* Reset Event Counter */
                    break;                   
                }
                case INT2:
                {
                    /* Store interrupt event count for INT2 */
                    intEventCount[0] = _INT_EventCout[INT2];
                    if(dataBuf[1]) _INT_EventCout[INT2] = 0; /* Reset Event Counter */
                    break;                   
                }
                default: intEventCount[0] =  (ERROR_CODE); break; /* Return error */
            }
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x81,5,TI_StatusCode);
            /* Send Event Counts to UART */
            UARTWriteData(intEventCount,4);
            break;
        }

        case INT_SET_HANDLER_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Enable/disable interrupt callback handlers */
            switch (dataBuf[0]) {
                case INT0:
                    intCallBack[INT0].intCallBackSet = (dataBuf[1]) ? TRUE : FALSE;
                    break;
                case INT1:
                    intCallBack[INT1].intCallBackSet = (dataBuf[1]) ? TRUE : FALSE;
                    break;
                case INT2:
                    intCallBack[INT2].intCallBackSet = (dataBuf[1]) ? TRUE : FALSE;
                    break;
                default:
                    TI_StatusCode = -1;
                    break;
            }
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x82,1,TI_StatusCode);
            break;
        }
    }
    return 0;
}


/**
 * @brief Process PWM commands and execute corresponding operations
 * @param pwmCommandState PWM command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles PWM operations including configuration, enable/disable, and frequency
 * setting. Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  PWMcmdCheck(enum TI_PWM_COMMAND_CODE pwmCommandState)
{
    /* Process PWM command based on command state */
    switch(pwmCommandState)
    {
        case PWM_CONFIG_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Process PWM configuration sub-commands */
            switch(dataBuf[1])
            {
                case 0:
                {   
                    /* Disable PWM output for specified channel */
                    TI_StatusCode = PWM_DISABLE(dataBuf[0]);      
                    break;
                }

                case 1:
                {
                    /* Reconstruct 32-bit frequency value from 4 bytes (big-endian) */
                    uint32_t freqValue = ((uint32_t)dataBuf[5])|((uint32_t)dataBuf[4] << 8) |((uint32_t)dataBuf[3] << 16)|
                    ((uint32_t)dataBuf[2] << 24);
                    /* Enable PWM output with specified frequency and duty cycle */
                    TI_StatusCode = PWM_ENABLE(dataBuf[0],freqValue,dataBuf[6]);
                    break;
                }
            }
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x80,1,TI_StatusCode);
            break;
        }
        default: return RETURN_FALSE;
    }
    
    return RETURN_TRUE;
}

/**
 * @brief Process CAN FD commands and execute corresponding operations
 * @param canCommandState CAN FD command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles CAN FD operations including getting default parameters, configuring
 * the interface, transmitting data, and reading received data. Processes data
 * from the global dataBuf and sends responses via UART.
 */
static int8_t  CANFDcmdCheck(enum TI_CANFD_COMMAND_CODE canCommandState)
{
    /* Check command state and process accordingly */
    switch (canCommandState) 
    {
        case CANFD_GET_DEFAULT_PARAMS_CMD_CODE:
        {
            /* Buffer to store CAN FD parameters */
            uint8_t _CANFD_ParamsBuff[52] = {0};
            TI_StatusCode = SystemP_SUCCESS;
            
            /* Get default CAN FD configuration parameters */
            if(CANFD_GetDefaultParams(_CANFD_ParamsBuff) != SystemP_SUCCESS)
            {
                DebugP_logError("Failed to read CAN FD default parameters\r\n");
                TI_StatusCode = SystemP_FAILURE;
                /* Send Error Code */
                UARTWriteCMD(&gUARTCmdObjects,0x80,1,TI_StatusCode);
            }
            else 
            {
                DebugP_logInfo("Successfully read CAN FD default parameters\r\n");
                /* Send success code and parameter data */
                UARTWriteCMD(&gUARTCmdObjects,0x80,53,TI_StatusCode);
                /* Send parameter data */
                UARTWriteData(_CANFD_ParamsBuff,52);
            }
            break;
        }

        case CANFD_CONFIG_CMD_CODE:
        {
            /* Buffer to store configuration parameters */
            uint32_t configCANFDBuff[13] = {0};
            TI_StatusCode = SystemP_SUCCESS;
            /* Calculate output capacity */
            size_t outCapacity = sizeof(configCANFDBuff) / sizeof(configCANFDBuff[0]); /* 13 */
            /* Convert uint8_t array to uint32_t array for configuration */
            int32_t words = u8_to_u32_convert(dataBuf, configCANFDBuff, sizeof(configCANFDBuff),outCapacity);
            /* Check if conversion was successful */
            if (words < 0) 
            {
                TI_StatusCode = SystemP_FAILURE; /* not enough space or invalid args */
            } 
            else 
            {
                /* If CANFD_Configure requires exactly 13 words, ensure words == 13 */
                if (words != (int32_t)outCapacity) 
                {
                    /* Decide: either reject or allow (if it accepts fewer with zero padding) */
                    /* Here we enforce exact 13: */
                    TI_StatusCode = SystemP_FAILURE;
                }
            }

            if (TI_StatusCode == SystemP_SUCCESS) 
            {
                /* Configure CAN FD */
                TI_StatusCode = CANFD_Configure(configCANFDBuff);
            }
            /* Send status code to uart */
            UARTWriteCMD(&gUARTCmdObjects,0x81,1,TI_StatusCode);
            break;
        }

        case CANFD_TX_OBJECT_CREATE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Construct 32-bit start message ID from 4 bytes (big-endian) */
            startMsgID_Tx = ((uint32_t)dataBuf[1] << 24) |
                                    ((uint32_t)dataBuf[2] << 16) |
                                    ((uint32_t)dataBuf[3] << 8)  |
                                    ((uint32_t)dataBuf[4]);
                                    
            /* Construct 32-bit end message ID from 4 bytes (big-endian) */
            endMsgID_Tx = ((uint32_t)dataBuf[5] << 24) |
                                    ((uint32_t)dataBuf[6] << 16) |
                                    ((uint32_t)dataBuf[7] << 8)  |
                                    ((uint32_t)dataBuf[8]);
            /*
                CANFD Data Length should be in the range of 8Bytes,16Bytes,
                20Bytes,24Bytes,32Bytes,48Bytes, and 64Bytes.    
            */    
            /* Extract data length from buffer */
            dataLen_Tx = dataBuf[9];
            /* Create CAN FD transmit message object */
            TI_StatusCode = CANFD_Transmit_MsgObjectCreate(dataBuf[0],startMsgID_Tx,endMsgID_Tx,dataLen_Tx);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x82,1,TI_StatusCode);
            break;
        }

        case CANFD_WRITE_DATA_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Transmit CAN FD data */
            TI_StatusCode = CANFD_TransmitData(dataBuf);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x83,1,TI_StatusCode);
            break;
        }
        
        case CANFD_RX_OBJECT_CREATE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;            
            /* Construct 32-bit start message ID from 4 bytes (big-endian) */
            startMsgID_Rx = ((uint32_t)dataBuf[1] << 24) |
                                    ((uint32_t)dataBuf[2] << 16) |
                                    ((uint32_t)dataBuf[3] << 8)  |
                                    ((uint32_t)dataBuf[4]);
                                    
            /* Construct 32-bit end message ID from 4 bytes (big-endian) */
            endMsgID_Rx = ((uint32_t)dataBuf[5] << 24) |
                                    ((uint32_t)dataBuf[6] << 16) |
                                    ((uint32_t)dataBuf[7] << 8)  |
                                    ((uint32_t)dataBuf[8]);
            /*
                CANFD Data Length should be in the range of 8Bytes,16Bytes,
                20Bytes,24Bytes,32Bytes,48Bytes, and 64Bytes.    
            */
            /* Extract data length from buffer */
            dataLen_Rx = dataBuf[9];
                                                
            /* Create CAN FD receive message object */
            CANFD_Recv_MsgObjectCreate(dataBuf[0],startMsgID_Rx,endMsgID_Rx,dataLen_Rx);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects,0x84,1,TI_StatusCode);
            break;
        }
        case CANFD_READ_DATA_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Receive CAN data into buffer (assumes CANFD_ReceiveData fills up to dataLen_Rx bytes) */
            gCANRxISRFlage = 0;
            TI_StatusCode = CANFD_ReceiveData(gCANReadBuff);
            /* Header: status + payload length (protocol uses length = payload + 1) */
            UARTWriteCMD(&gUARTCmdObjects,0x85,(uint16_t)(dataLen_Rx + 1u),TI_StatusCode);
            /* Payload: the actual CAN data */
            UARTWriteData(gCANReadBuff,(uint16_t)dataLen_Rx);
            /* Clear receive buffer */
            memset(gCANReadBuff,0,sizeof(gCANReadBuff));
            break;
        }

        case CANFD_WRITE_CANCEL_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Get transmit message handle for cancellation */
            CANFD_MsgObjHandle txHandle = CANFD_getTX_Msg_Handle();  /* prototype must be visible and match */
            if (txHandle == NULL) 
            {
                TI_StatusCode = SystemP_FAILURE;   /* error */
            } 
            else 
            {
                /* Cancel CAN FD transmit */
                TI_StatusCode = CANFD_writeCancel(txHandle);
            }
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x86,0x01,TI_StatusCode);
            break;
        }

        case CANFD_VALID_DATA_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Validate CAN FD data length */
            TI_StatusCode = CANFD_ValidateDataLen(dataBuf[0]);
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x87,0x01,TI_StatusCode);
            break;
        }

        case CANFD_GET_FAILURE_REASON_CMD_CODE:
        {
            uint8_t errorCode[1] = {0};
            TI_StatusCode = SystemP_SUCCESS;
            /* Get last CAN FD operation failure reason */
            errorCode[0] = CANFD_GetFailureReason();
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x88,0x02,TI_StatusCode);
            /* Send error code to UART */
            UARTWriteData(errorCode,1);
            break;
        }

        case CANFD_CLOSE_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Close CAN FD peripheral */
            TI_StatusCode = CANFD_Close();
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x89,0x01,TI_StatusCode);
            break;
        }

        case CANFD_DLT_TX_OBJECT_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Delete CAN FD transmit message object */
            TI_StatusCode = CANFD_deleteTXObject();
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x8A,0x01,TI_StatusCode);
            break;
        }
        
        case CANFD_DLT_RX_OBJECT_CMD_CODE:
        {
            TI_StatusCode = SystemP_SUCCESS;
            /* Delete CAN FD receive message object */
            TI_StatusCode = CANFD_deleteRXObject();
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x8B,0x01,TI_StatusCode);
            break;
        }       
        default: return RETURN_FALSE;
    }

    return RETURN_TRUE;
    
}

/**
 * @brief Process DDR commands and execute corresponding operations
 * @param ddrCommandState DDR command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles DDR operations including memory testing, read, write operations.
 * Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  DDRcmdCheck(enum TI_DDR_MEM_COMMAND_CODE ddrCommandState)
{
    uint32_t *offset;
    uint32_t blockSize;
    uint8_t readData[128] = {0};
    switch(ddrCommandState)
    {
        case DDR_MEM_TEST_CMD_CODE:
            TI_StatusCode = SystemP_SUCCESS;
            
            offset = (uint32_t*)(((uint32_t)dataBuf[0] << 24) |
                                    ((uint32_t)dataBuf[1] << 16) |
                                    ((uint32_t)dataBuf[2] << 8)  |
                                    ((uint32_t)dataBuf[3]));

            blockSize = ((uint32_t)dataBuf[4] << 24) |
                                    ((uint32_t)dataBuf[5] << 16) |
                                    ((uint32_t)dataBuf[6] << 8)  |
                                    ((uint32_t)dataBuf[7]);
            
            /* Perform DDR Memory Testing */
            TI_StatusCode = DDR_Memtester(offset, blockSize);

            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x80,0x01,TI_StatusCode);
            break;

        case DDR_MEM_READWRITE_CMD_CODE:
            TI_StatusCode = SystemP_SUCCESS;
            offset = (uint32_t*)(((uint32_t)dataBuf[0] << 24) |
                                    ((uint32_t)dataBuf[1] << 16) |
                                    ((uint32_t)dataBuf[2] << 8)  |
                                    ((uint32_t)dataBuf[3]));

            /* Perform DDR Read Write Operation */
            TI_StatusCode = DDR_ReadWrite(offset, dataBuf[4], &dataBuf[5], readData);

            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x81,0x01+dataBuf[4],TI_StatusCode);
            if(dataBuf[4] > 0)
            {
                UARTWriteData(readData, dataBuf[4]);
            }
            break;

        default:
            return RETURN_FALSE;
    }
    return RETURN_TRUE;
}

/**
 * @brief Process EMMC commands and execute corresponding operations
 * @param emmcCommandState EMMC command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles EMMC operations including memory testing, read, write operations.
 * Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  EMMCcmdCheck(enum TI_EMMC_MEM_COMMAND_CODE emmcCommandState)
{
    uint32_t offset = 0;
    /* Defining 512 bytes of read buffer as read in EMMC happens block wise. */
    uint8_t readData[512] = {0}; 
    switch(emmcCommandState)
    {
        case EMMC_MEM_READWRITE_CMD_CODE:
            TI_StatusCode = SystemP_SUCCESS;

            offset = (((uint32_t)dataBuf[0] << 24) |
                                    ((uint32_t)dataBuf[1] << 16) |
                                    ((uint32_t)dataBuf[2] << 8) |
                                    ((uint32_t)dataBuf[3]));

            /* Perform EMMC Read Write Operation */
            TI_StatusCode = EMMC_ReadWrite(offset, dataBuf[4], &dataBuf[5], readData);
            
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x80,0x01+dataBuf[4],TI_StatusCode);
            if(dataBuf[4] > 0)
            {
                UARTWriteData(readData, dataBuf[4]);
            }
            break;
        default:
            return RETURN_FALSE;
    }
    return RETURN_TRUE;
}


/**
 * @brief Process OSPI commands and execute corresponding operations
 * @param ospiCommandState OSPI command code to execute
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Handles OSPI operations including memory testing, read, write operations.
 * Processes data from the global dataBuf and sends responses via UART.
 */
static int8_t  OSPIcmdCheck(enum TI_OSPI_FLASH_COMMAND_CODE ospiCommandState)
{
    uint32_t offset;
    uint8_t readData[128] = {0};
    switch(ospiCommandState)
    {
        case OSPI_FLASH_READWRITE_CMD_CODE:
            TI_StatusCode = SystemP_SUCCESS;

            offset = (((uint32_t)dataBuf[0] << 24) |
                                    ((uint32_t)dataBuf[1] << 16) |
                                    ((uint32_t)dataBuf[2] << 8) |
                                    ((uint32_t)dataBuf[3]));

            /* Perform EMMC Read Write Operation */
            TI_StatusCode = OSPI_ReadWrite(offset, dataBuf[4], &dataBuf[5], readData);
            
            /* Send status code to UART */
            UARTWriteCMD(&gUARTCmdObjects, 0x80,0x01+dataBuf[4],TI_StatusCode);
            if(dataBuf[4] > 0)
            {
                UARTWriteData(readData, dataBuf[4]);
            }
            break;
        default:
            return RETURN_FALSE;
    }
    return RETURN_TRUE;
}

/**
 * @brief Main function command dispatcher
 * @param funState Function code indicating which command processor to call
 * @return RETURN_TRUE on success, RETURN_FALSE on invalid command
 * 
 * Extracts command code from UART header and dispatches to appropriate
 * command processing function based on function type (I2C, SPI, UART, GPIO,
 * Interrupt, PWM, DAC, CAN FD, or Power).
 */
static int8_t funcCheck(enum TI_FUNCTION_CODE funState)
{
    /* Extract command code from UART header buffer */
    uint8_t cmdCode = gUARTCmdObjects.headBuf[UART_CMD_CODE_ID];

    /* Dispatch to appropriate command processor based on function code */
    switch (funState)
    {
        case I2C_FN_CODE:
            /* Handle I2C protocol commands */
            I2CcmdCheck(cmdCode);
            break;

        case SPI_FN_CODE:
            /* Handle SPI protocol commands */
            SPIcmdCheck(cmdCode);
            break;

        case UART_FN_CODE:
            /* Handle UART protocol commands */
            UARTcmdCheck(cmdCode);
            break;

        case GPIO_FN_CODE:
            /* Handle GPIO protocol commands */
            GPIOcmdCheck(cmdCode);
            break;

        case INT_FN_CODE:
            /* Handle interrupt protocol commands */
            INTcmdCheck(cmdCode);
            break;

        case PWM_FN_CODE:
            /* Handle PWM protocol commands */
            PWMcmdCheck(cmdCode);
            break;
        case CANFD_FN_CODE:
            /* Handle CAN FD protocol commands */
            CANFDcmdCheck(cmdCode);
            break;
        case DDR_MEM_FN_CODE:
            /* Handle DDR Memory protocol commands */
            DDRcmdCheck(cmdCode);
            break;
        case EMMC_MEM_FN_CODE:
            /* Handle DDR Memory protocol commands */
            EMMCcmdCheck(cmdCode);
            break;
        case OSPI_FLASH_FN_CODE:
            /* Handle DDR Memory protocol commands */
            OSPIcmdCheck(cmdCode);
            break;
        default:
            /* Log error for invalid function code */
            DebugP_logError("Invalid function code: %d", funState);
            return RETURN_FALSE;
    }

    return RETURN_TRUE;
}

/**
 * @brief Main UART command parsing and processing function
 * @param uartPayload Pointer to received UART data payload
 * @return RETURN_TRUE on success, RETURN_FALSE on failure
 * 
 * Validates UART payload, extracts header information, validates signature,
 * and dispatches commands to appropriate protocol handlers. This is the
 * main entry point for processing UART commands from the host.
 */
int8_t gParseUARTCMD(BYTE* uartPayload)
{
    /* Validate input parameter */
    if (!uartPayload)
    {
        DebugP_logError("UART payload is NULL.\r\n");
        return RETURN_FALSE;
    }


    /* Copy UART Payload into header buffer for parsing */
    memcpy(gUARTCmdObjects.headBuf, uartPayload, UART_CMD_HEADER_LEN); /* 9-byte header */

    /* Validate TI Board Diagnostic signature (Must be 'T' 'I') */
    if ((gUARTCmdObjects.headBuf[0] == SIGN_T_CHAR_HEX) && (gUARTCmdObjects.headBuf[1] == SIGN_I_CHAR_HEX))
    {
        gUARTCmdObjects.valid_sign = VALID;

        /* Extract data length from header */
        uint8_t dataLen = gUARTCmdObjects.headBuf[UART_DATA_LEN_ID];
        /* Store data length in UART object */
        gUARTCmdObjects.uarti2cparam.uI2CDatabufLen = dataLen;

        /* Validate and process data payload */
        if (dataLen > 0)
        {
            memset(dataBuf, 0, MAX_CMD_DATA_LEN);
            /* Copy data payload from UART packet */
            memcpy(dataBuf, &uartPayload[UART_CMD_HEADER_LEN], dataLen);
        }
        else
        {
            DebugP_logError("Invalid data length: %d\r\n", dataLen);
            return RETURN_FALSE;
        }

        /* Extract function code from header and dispatch to appropriate handler */
        gFun_Code = gUARTCmdObjects.headBuf[UART_CMD_FUN_CODE_ID];
        /* Check function code and Process further command */
        funcCheck(gFun_Code);
    }
    else
    {
        /* Invalid signature - reject command */
        gUARTCmdObjects.valid_sign = INVALID;
        DebugP_logError("Invalid Board Diagnostic signature - [UART_ERROR_CODE]: %d\r\n", SIGNATURE_ERROR);
        return RETURN_FALSE;
    }
    return RETURN_TRUE;
}
