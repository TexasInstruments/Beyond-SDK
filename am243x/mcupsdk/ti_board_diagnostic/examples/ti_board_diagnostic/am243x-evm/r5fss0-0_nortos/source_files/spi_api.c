/**
 * @file spi_api.c
 * @brief SPI communication API implementation for TI AM2432 Board Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements SPI (Serial Peripheral Interface) communication functions
 * for the TI AM2432 Board Diagnostic project, supporting multiple SPI channels with
 * configurable clock speeds, frame formats, and chip select options.
 */

#include "../header_files/common_extern.h"

/* SPI message size for transfer operations */
#define APP_MCSPI_MSGSIZE               (100U)
/* Number of transfer iterations for loopback test */
#define APP_MCSPI_TRANSFER_LOOPCOUNT    (10U)

/* External SPI channel configuration */
extern MCSPI_ChConfig *gConfigMcspiChCfg[];

/* SPI communication buffers */
uint8_t gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t gMcspiRxBuffer[APP_MCSPI_MSGSIZE];

/**
 * @brief Pinmux configuration for SPI Channel 0 (SPI2)
 * Maps SPI2 pins to UART pins for multi-function usage
 */
static Pinmux_PerCfg_t SPI_CH0_PMUX_CONFIG[] = {

        /* SPI0 pin config */
    /* SPI0_CLK -> SPI0_CLK (D13) */
    {
        PIN_SPI0_CLK,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SPI0 pin config */
    /* SPI0_D0 -> SPI0_D0 (A13) */
    {
        PIN_SPI0_D0,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SPI0 pin config */
    /* SPI0_D1 -> SPI0_D1 (A14) */
    {
        PIN_SPI0_D1,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
        
            /* SPI0_CS0 pin config */
    /* SPI0_CS0 -> SPI0_CS0 (D12) */
    {
        PIN_SPI0_CS0,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
        
    {PINMUX_END, PINMUX_END}   
};

/**
 * @brief Pinmux configuration for SPI Channel 1 CS0
 * Configures SPI1_CS0 pin for chip select functionality
 */
static Pinmux_PerCfg_t SPICH1_CS0_PMUX_CONFIG[] = {
     /* MCU_SPI0 pin config */
    /* MCU_SPI0_CLK -> MCU_SPI0_CLK (E6) */
    {
        PIN_MCU_SPI0_CLK,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* MCU_SPI0 pin config */
    /* MCU_SPI0_D0 -> MCU_SPI0_D0 (E7) */
    {
        PIN_MCU_SPI0_D0,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* MCU_SPI0 pin config */
    /* MCU_SPI0_D1 -> MCU_SPI0_D1 (B6) */
    {
        PIN_MCU_SPI0_D1,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },

                    /* MCU_SPI0_CS0 pin config */
    /* MCU_SPI0_CS0 -> MCU_SPI0_CS0 (D6) */
    {
        PIN_MCU_SPI0_CS0,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* End of configuration */
    {PINMUX_END, PINMUX_END}   
};

/**
 * @brief Configure SPI channel with specified parameters
 * @param ch SPI channel number (CONFIG_SPI_CH0 or CONFIG_SPI_CH1)
 * @param nCS Chip select number (0 for CS0, 1 for CS1)
 * @param mode SPI frame format (POL0/PHASE0, POL0/PHASE1, etc.)
 * @param clkFreq SPI clock frequency
 * @param cs_Type Chip select type (TRUE/FALSE for manual control)
 * @param cspolarity Chip select polarity (active high/low)
 * @return SystemP_SUCCESS on success, error code on failure
 */
int32_t SPI_Config(SPI_CH ch, BYTE nCS, SPI_FrameFormat mode, SPI_Clock clkFreq, bool cs_Type, SPI_CSPolarity cspolarity)
{
    switch (ch)
    {
        case CONFIG_SPI_CH0:
        {   
            ti_status = MCSPI_STATUS_SUCCESS;

            /* Set pin multiplexing configuration for SPI Channel 0 */  
            Pinmux_config(SPI_CH0_PMUX_CONFIG, PINMUX_DOMAIN_ID_MAIN);

            /* Close any previously opened SPI Channel 0 */    
            MCSPI_lld_deInit(gMcspiHandle[CONFIG_SPI_CH0]);

            /* Clear driver open flag */
            gUARTCmdObjects.uartspiparam.uSPIHandle = 0;

            DebugP_logInfo("[MAIN_SPI0]: SPI Clock - %d\n", clkFreq);

            /* Set SPI configuration parameters */
            gMcspiInitObject[CONFIG_SPI_CH0].chObj[0].chCfg->bitRate = clkFreq;
            gMcspiInitObject[CONFIG_SPI_CH0].chObj[0].chCfg->frameFormat = mode;
            gMcspiInitObject[CONFIG_SPI_CH0].chObj[0].chCfg->csPolarity = cspolarity;
            
            /* Configure SPI handle and base address */
            gMcspiHandle[CONFIG_SPI_CH0] = &gMcspiObject[CONFIG_SPI_CH0];
            gMcspiHandle[CONFIG_SPI_CH0]->state = MCSPI_STATE_RESET;
            gMcspiHandle[CONFIG_SPI_CH0]->baseAddr = CSL_MCSPI0_CFG_BASE;
            gMcspiHandle[CONFIG_SPI_CH0]->hMcspiInit = &gMcspiInitObject[CONFIG_SPI_CH0]; 
            
            /* Set channel and chip select configuration */
            gMcspiInitObject[CONFIG_SPI_CH0].chObj[0].chCfg->slvCsSelect = MCSPI_SLV_CS_SELECT_0;
            gMcspiInitObject[CONFIG_SPI_CH0].chObj[0].chCfg->chNum = MCSPI_CHANNEL_0;

            /* Set chip select control type */
            gUARTCmdObjects.uartspiparam.uSPI_CSType = cs_Type;

            /* Initialize SPI driver */
            ti_status += MCSPI_lld_init(gMcspiHandle[CONFIG_SPI_CH0]);
           
            if (ti_status != MCSPI_STATUS_SUCCESS)
            {
                DebugP_logError("[MAIN_SPI0]: Driver Open Fail \n");
                return ti_status;
            }

            /* Set driver open flag */
            gUARTCmdObjects.uartspiparam.uSPIHandle = 1;
            DebugP_logInfo("[MAIN_SPI0]: Driver Opened Successfully \n");
            
            break;      
        }

        case CONFIG_SPI_CH1:
        {
            ti_status = MCSPI_STATUS_SUCCESS;

            /* Set pin multiplexing configuration for SPI Channel 1 */
            Pinmux_config(SPICH1_CS0_PMUX_CONFIG, PINMUX_DOMAIN_ID_MCU);

            /* Close any previously opened SPI Channel 1 */
            MCSPI_lld_deInit(gMcspiHandle[CONFIG_SPI_CH1]);

            /* Clear driver open flag */
            gUARTCmdObjects.uartspiparam.uSPIHandle = 0;
            DebugP_logInfo("[MCU_SPI0]: SPI Clock - %d\n", clkFreq);
            
            /* Set SPI configuration parameters */
            gMcspiInitObject[CONFIG_SPI_CH1].chObj[0].chCfg->bitRate = clkFreq;
            gMcspiInitObject[CONFIG_SPI_CH1].chObj[0].chCfg->frameFormat = mode;
            gMcspiInitObject[CONFIG_SPI_CH1].chObj[0].chCfg->csPolarity = cspolarity;
            
            /* Configure SPI handle and base address */
            gMcspiHandle[CONFIG_SPI_CH1] = &gMcspiObject[CONFIG_SPI_CH1];
            gMcspiHandle[CONFIG_SPI_CH1]->state = MCSPI_STATE_RESET;
            gMcspiHandle[CONFIG_SPI_CH1]->baseAddr = CSL_MCU_MCSPI0_CFG_BASE;
            gMcspiHandle[CONFIG_SPI_CH1]->hMcspiInit = &gMcspiInitObject[CONFIG_SPI_CH1];

            /* Set chip select control type */
            gUARTCmdObjects.uartspiparam.uSPI_CSType = cs_Type;

            /* Initialize SPI driver */
            ti_status = MCSPI_lld_init(gMcspiHandle[CONFIG_SPI_CH1]);
            if (ti_status != MCSPI_STATUS_SUCCESS)
            {
                DebugP_logError("[MCU_SPI0]: Driver Open Fail \n");
                return -1;
            }

            /* Set driver open flag */
            gUARTCmdObjects.uartspiparam.uSPIHandle = 1;
            DebugP_logInfo("[MCU_SPI0]: Driver Opened Successfully \n");
            break;
        }

        default:
        {
            break;
        }
    }
    return SystemP_SUCCESS;
}

/**
 * @brief Perform simultaneous SPI read and write operation
 * @param uSPIConfigParam Pointer to UART SPI configuration parameters
 * @param txData Pointer to transmit data buffer
 * @param rxData Pointer to receive data buffer
 * @param byteCount Number of bytes to transfer
 * @return MCSPI_STATUS_SUCCESS on success, error code on failure
 */
int32_t SPI_ReadWrite(UART_OBJ* uSPIConfigParam, void* txData, void* rxData, WORD byteCount)
{
    ti_status = MCSPI_STATUS_SUCCESS;
    MCSPI_ExtendedParams spiExtendParam;
    uint32_t timeout = MCSPI_WAIT_FOREVER;

    /* Define channel string for debug logs */
    const char* channel_str = (uSPIConfigParam->uartspiparam.uSPIChannel == CONFIG_SPI_CH0) ? "MAIN_SPI0" : "MCU_SPI0";

    /* Validate SPI driver handle */
    if (!uSPIConfigParam->uartspiparam.uSPIHandle)
    {
        DebugP_logError("[%s] : Driver Is Not Opened...\n", channel_str);
        return -1;        
    }

    /* Configure channel based on SPI channel selection */
    if (!uSPIConfigParam->uartspiparam.uSPIChannel)
    {
        spiExtendParam.channel = gConfigSpiCh0ChCfg[0].chNum;
    } 
    else if (uSPIConfigParam->uartspiparam.uSPIChannel == CONFIG_SPI_CH1)
    {
        spiExtendParam.channel = gConfigSpiCh1ChCfg[0].chNum;
    }

    /* Set SPI transfer parameters */
    spiExtendParam.csDisable = uSPIConfigParam->uartspiparam.uSPI_CSType;
    spiExtendParam.dataSize = 8; /* Keep the data size default */

    /* Perform SPI read/write operation */
    ti_status += MCSPI_lld_readWrite(gMcspiHandle[uSPIConfigParam->uartspiparam.uSPIChannel], \
                                                txData, \
                                                rxData, \
                                                byteCount, \
                                                timeout, \
                                                &spiExtendParam);
    if (ti_status != MCSPI_STATUS_SUCCESS)
    {
        DebugP_logError("[%s] : SPI Read/Write Fails ...\n", channel_str);
        return ti_status;
    }
    return MCSPI_STATUS_SUCCESS;
}
