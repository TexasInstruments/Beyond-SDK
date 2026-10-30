/**
 * @file uart_api.c
 * @brief UART communication API implementation for TI AM2432 Board Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements UART (Universal Asynchronous Receiver/Transmitter) 
 * communication functions for the TI AM2432 Board Diagnostic project, supporting
 * multiple baud rates, parity settings, and data configurations.
 */

#include "../header_files/common_extern.h"
#include "../configurations/uart_driver.h"

/* UART channel definitions */
#define SOC_UART0_CH0                        0U

/* UART Enhanced Control Register offsets and bits */
#define UART_ECR_OFFSET             0x90U
#define UART_ECR_RX_DISABLE           3U

/* External UART initialization objects */
//extern UARTLLD_InitObject gUartParams[2];

/* UART configuration and transaction objects */
USER_CONFIG_PARA uart_userConfig[2];
UART_Transaction uartWriteParam;
UART_Transaction uartReadParam;
UARTLLD_InitHandle sysTick_UART;

/* UART timeout value (5 sec) */
uint32_t uTimeout = (5 * 1000) ;

/* Forward declaration */
uint32_t UART_Read_RX_cnt(uint8_t channel);

/**
 * @brief Pinmux configuration for UART Channel 0
 * Maps UART0 pins to their respective GPIO pins for communication
 */
static Pinmux_PerCfg_t UART_CH0_PMUX_CONFIG[] = {
    
    /* UART0 pin config */
    /* UART0_RXD (D15) */
    {
        PIN_UART0_RXD,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    }, 

    /* UART0 pin config */
    /* UART0_TXD (C16) */
    {
        PIN_UART0_TXD,
        ( PIN_MODE(0) | PIN_PULL_DISABLE )
    },

    {PINMUX_END, PINMUX_END}   
};

/**
 * @brief Configure UART channel with specified parameters
 * @param ch UART channel number (SOC_UART0_CH0)
 * @param baud UART baud rate (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
 * @param parity Parity setting (0=none, 1=odd, 2=even)
 * @param dataLen Data length in bits (typically 8)
 * @param stopBITLen Stop bit length (typically 1)
 * @return UART_STATUS_SUCCESS on success, error code on failure
 */
int UART_CONFIG(UART_CH ch, UART_Baudrate baud, BYTE parity, BYTE dataLen, BYTE stopBITLen)
{
    uint32_t baudrate_t = 0;
    uint32_t opermode_t = 0; /* Default mode - 16x oversampling */


    ti_status = SystemP_SUCCESS;

    /* Configure baud rate and operating mode */
    switch (baud)
    {
        case UART_BAUD_9600:
            baudrate_t = 9600;
            opermode_t = UART_OPER_MODE_16X;
            break;

        case UART_BAUD_19200:
            baudrate_t = 19200;
            opermode_t = UART_OPER_MODE_16X;
            break;

        case UART_BAUD_38400:
            baudrate_t = 38400;
            opermode_t = UART_OPER_MODE_16X;
            break;

        case UART_BAUD_57600:
            baudrate_t = 57600;
            opermode_t = UART_OPER_MODE_16X;
            break;

        case UART_BAUD_115200:
            baudrate_t = 115200;
            opermode_t = UART_OPER_MODE_16X;                           /* Changed from 13x to 16x */
            break;

        case UART_BAUD_230400:
            baudrate_t = 230400;
            opermode_t = UART_OPER_MODE_16X;                       /* Changed from 13x to 16x */
            break;

        case UART_BAUD_460800:
            baudrate_t = 460800;
            opermode_t = UART_OPER_MODE_13X;                       /* Changed from 16x to 13x */
            break;            

        case UART_BAUD_921600:
            baudrate_t = 921600;
            opermode_t = UART_OPER_MODE_13X;                       /* Changed from 16x to 13x */
            break;            
        default:
            baudrate_t = 115200;
            opermode_t = UART_OPER_MODE_16X;                           /* Changed from 13x to 16x */
            break;
    }

    DebugP_logInfo("UART baudrate configured to %d.\r\n", baudrate_t);

    /* Configure specified UART channel */
    switch (ch) 
    {

        case SOC_UART0_CH0:
        {
            /* Enable UART clock */
            ti_status += SOC_moduleClockEnable(TISCI_DEV_UART0, 1);
            ti_status += SOC_moduleSetClockFrequency(TISCI_DEV_UART0,TISCI_DEV_UART0_FCLK_CLK,48000000);
            DebugP_logInfo("UART Clock set to %d MHz.\r\n", 48000000);

            /* Close any previously opened UART handle */
            UART_lld_deInit(gUart0HandleLld[0]);

            /* Clear handle flag */
            gUARTCmdObjects.uartuartparam.uUARTHandle = 0;

            /* Configure pin multiplexing for UART Channel 0 */
            Pinmux_config(UART_CH0_PMUX_CONFIG, PINMUX_DOMAIN_ID_MAIN);

            /* Initialize UART0 driver - set parameters */
            gUart0InitObject[0].baudRate = baudrate_t;
            gUart0InitObject[0].operMode = opermode_t;
            gUart0InitObject[0].parityType = parity;
            gUart0InitObject[0].dataLength = dataLen;
            gUart0InitObject[0].stopBits = stopBITLen;

            /* Configure UART0 driver handle */
            gUart0HandleLld[0] = &gUart0Object[0];
            gUart0HandleLld[0]->state = UART_STATE_RESET;
            gUart0HandleLld[0]->baseAddr = CSL_UART0_BASE;
            gUart0HandleLld[0]->hUartInit = &gUart0InitObject[0];

            /* Initialize UART driver */
            ti_status += UART_lld_init(gUart0HandleLld[0]);

            if (ti_status != UART_STATUS_SUCCESS)
            {
                return ti_status;
            }
            DebugP_logInfo("UART instance initialized successfully.\r\n");

            /* Set handle flag to indicate driver is open */
            gUARTCmdObjects.uartuartparam.uUARTHandle = 1;
            return UART_STATUS_SUCCESS;
            break;
        }
        default:
            break;
    }
    return SystemP_SUCCESS;    
}

/**
 * @brief Write data to UART channel
 * @param uUARTConfigParam Pointer to UART UART configuration parameters
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer to write
 * @return SystemP_SUCCESS on success, error code on failure
 */
int UART_Write(UART_OBJ* uUARTConfigParam, BYTE nBytes, const void *data)
{
    uint8_t uartWriteDataBuf[255];
    ti_status = SystemP_SUCCESS;
    
    /* Validate UART handle */
    if (uUARTConfigParam->uartuartparam.uUARTHandle)
    {
        /* Clear write buffer */
        memset(uartWriteDataBuf, 0, 255);

        /* Initialize UART write transaction */
        UART_lld_Transaction_init(&uartWriteParam);
        uartWriteParam.timeout = 1000;
        uartWriteParam.buf = uartWriteDataBuf;
        memcpy(uartWriteDataBuf, data, nBytes);
        uartWriteParam.count = nBytes;

        DebugP_logInfo("UART Write Started...\r\n");

        /* Perform UART write operation */
        ti_status += UART_lld_write(gUart0HandleLld[uUARTConfigParam->uartuartparam.uUARTChannel], uartWriteParam.buf,
                    uartWriteParam.count, uartWriteParam.timeout, NULL);

        if (ti_status != SystemP_SUCCESS)
        {
            DebugP_logError("UART Fails to Write Data \n");
            return ti_status;
        }
        DebugP_logInfo("UART Write Success.\r\n");
    }
    return SystemP_SUCCESS;
}

/**
 * @brief Read data from UART channel
 * @param uUARTConfigParam Pointer to UART UART configuration parameters
 * @param nBytes Number of bytes to read
 * @param data Pointer to buffer to store read data
 * @return SystemP_SUCCESS on success, error code on failure
 */
int UART_Read(UART_OBJ* uUARTConfigParam, BYTE nBytes, void *data)
{
    ti_status = SystemP_SUCCESS;
    uartReadParam.timeout = uTimeout;

    /* Validate UART handle */
    if (uUARTConfigParam->uartuartparam.uUARTHandle)
    {
        /* Initialize UART read transaction */
        UART_Transaction_init(&uartReadParam);
        uartReadParam.timeout = uTimeout;
        uartReadParam.buf = data;
        uartReadParam.count = nBytes;

        DebugP_logInfo("UART Read Started.\r\n");
        
        /* Perform UART read operation */
        ti_status += UART_lld_read(gUart0HandleLld[uUARTConfigParam->uartuartparam.uUARTChannel], uartReadParam.buf, uartReadParam.count,
                    uartReadParam.timeout, NULL);

        if (ti_status != SystemP_SUCCESS)
        {
            DebugP_logError("UART Fails to Read Data \n");
            return ti_status;
        }
        DebugP_logInfo("UART Read Success.\r\n");
    }

    return SystemP_SUCCESS;   
}

/**
 * @brief Disable UART receiver for specified channel
 * @param channel UART channel number
 * @return SystemP_SUCCESS on success, error code on failure
 */
int UART_DISABLE_RECEIVER(uint8_t channel)
{
    ti_status = SystemP_SUCCESS;
    uintptr_t baseAddr = gUart0HandleLld[channel]->baseAddr;
    
    /* Read current ECR register value */
    uint32_t regValue = HW_RD_REG32(baseAddr + UART_ECR_OFFSET);
    
    /* Toggle RX disable bit */
    regValue ^= (1U << UART_ECR_RX_DISABLE);
    
    /* Write modified value back to ECR register */
    HW_WR_REG32((baseAddr + UART_ECR_OFFSET), regValue);
    return ti_status;
}

/**
 * @brief Get number of bytes available in UART RX FIFO
 * @param channel UART channel number
 * @return Number of bytes in RX FIFO
 */
int UART_GET_RX_CNT(uint8_t channel)
{
    return HW_RD_REG32((gUart0HandleLld[channel]->baseAddr + UART_RXFIFO_LVL));
}

/**
 * @brief Set UART operating mode for specified channel
 * @param channel UART channel number
 * @param mode Operating mode to set
 * @return SystemP_SUCCESS on success, error code on failure
 */
int UART_SET_MODE(uint8_t channel, uint32_t mode)
{
    ti_status = SystemP_SUCCESS;
    ti_status += UART_operatingModeSelect(gUart0HandleLld[channel]->baseAddr, mode);
    char *mode_val = "";
    switch (mode)
    {
        case UART_OPER_MODE_16X:
            mode_val = "16X Oversampling";
            break;
        case UART_OPER_MODE_SIR:
            mode_val = "SIR (Serial Infrared)";
            break;
        case UART_OPER_MODE_16X_AUTO_BAUD:
            mode_val = "16X Auto Baud";
            break;
        case UART_OPER_MODE_13X:
            mode_val = "13X Oversampling";
            break;
        case UART_OPER_MODE_MIR:
            mode_val = "MIR (Manchester Infrared)";
            break;
        case UART_OPER_MODE_FIR:
            mode_val = "FIR (Fast Infrared)";
            break;
        case UART_OPER_MODE_CIR:
            mode_val = "CIR (Carrier Infrared)";
            break;
        case UART_OPER_MODE_DISABLED:
            mode_val = "Disabled";
            break;
    }
    DebugP_logInfo("UART Mode set to %s.\r\n", mode_val);
    return ti_status;  
}




