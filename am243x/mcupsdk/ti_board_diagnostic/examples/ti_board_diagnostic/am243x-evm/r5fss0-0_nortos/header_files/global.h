/**************************************************************************************************
 * @file global.h
 * @brief Global type definitions, constants, and data structures for the Board Diagnostic project
 * @author TI Board Diagnostic Team
 * @version 1.0
 * @date 2026
 * 
 * This header file contains global type definitions, constants, enumerations, and data structures
 * used throughout the TI AM2432 Board Diagnostic project.
 **************************************************************************************************/

#ifndef GLOBAL_H
#define GLOBAL_H

/*--------------------------------- STANDARD LIBRARY INCLUDES -----------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/**************************************************************************************************
 *                              CUSTOM TYPE DEFINITIONS
 **************************************************************************************************/

/*----------------------------------- BASIC DATA TYPES ------------------------------------------*/
/**
 * @brief 8-bit unsigned integer type
 */
typedef uint8_t BYTE;

/**
 * @brief 8-bit signed integer type
 */
typedef int8_t S_BYTE;

/**
 * @brief 32-bit unsigned integer type
 */
typedef uint32_t WORD;

/*---------------------------------- ADDRESS TYPES ---------------------------------------------*/
/**
 * @brief Register address type
 */
typedef uint32_t regAddress;

/*------------------------------------ I2C TYPES ----------------------------------------------*/
/**
 * @brief I2C channel type
 */
typedef uint8_t I2C_Channel;

/**
 * @brief I2C address length type
 */
typedef uint8_t I2C_AddressLength;

/**
 * @brief I2C speed type
 */
typedef uint8_t I2C_Speed;

/**
 * @brief I2C address type
 */
typedef uint16_t I2C_Address;

/*------------------------------------ UART TYPES ----------------------------------------------*/
/**
 * @brief UART channel type
 */
typedef uint8_t UART_CH;

/**
 * @brief UART baudrate type
 */
typedef uint32_t UART_Baudrate;

/*------------------------------------ SPI TYPES -----------------------------------------------*/
/**
 * @brief SPI channel type
 */
typedef uint8_t SPI_CH;

/**
 * @brief SPI frame format type
 */
typedef uint8_t SPI_FrameFormat;

/**
 * @brief SPI clock type
 */
typedef uint32_t SPI_Clock;

/**
 * @brief SPI chip select polarity type
 */
typedef uint8_t SPI_CSPolarity;

#define UART_CMD_HEADER_LEN               9U
#define CANFD_READ_DATA_LEN               64U

#define MAX_PWM_PINS 3U

/*------------------------------------ STATUS CONSTANTS ----------------------------------------*/
/**
 * @brief Active status flag
 */
#define ACTIVE   1

/**
 * @brief Inactive status flag
 */
#define NO_ACTIVE 0

/**
 * @brief Invalid status flag
 */
#define INVALID   -1

/**
 * @brief Valid status flag
 */
#define VALID      1

/*------------------------------------ I2C CONSTANTS ------------------------------------------*/

#define SOC_I2C0_ADDRESS_LEN_7BIT   0
#define SOC_I2C0_ADDRESS_LEN_10BIT  1

/* I2C Data Sizes */
#define I2C_8BIT_DATASIZE        1
#define I2C_16BIT_DATASIZE       2
#define I2C_24BIT_DATASIZE       3
#define I2C_32BIT_DATASIZE       4


/* UART BaudRates */
#define UART_BAUD_9600      0U
#define UART_BAUD_19200     1U
#define UART_BAUD_38400     2U
#define UART_BAUD_57600     3U
#define UART_BAUD_115200    4U
#define UART_BAUD_230400    5U
#define UART_BAUD_460800    6U
#define UART_BAUD_921600    7U

/* I2C Timeout */
#define I2C_TIMEOUT_uS             10000U

/* CAN ERROR CODES */
#define CANFD_IS_ALREADY_OPEN_ERR                         (1U)
#define CANFD_INVALID_PARAMS_ERR                          (2U)
#define CANFD_OUT_OF_RANGE_ERR                            (3U)
#define CANFD_INVALID_DATA_LEN_ERR                        (4U)

/* ------------------------------------ HEADER --------------------------------- */

/**
 * @brief User Configuration Parameters
 */
typedef struct
{
    /*Peripheral Driver Status */
    uint8_t driver_status;
    /*Peripheral Channel*/
    BYTE channel_status;
    /*Peripheral Type*/
    BYTE type;
    /*I2C Address Length*/
    BYTE addLen;
    /*I2C Speed */
    BYTE clockRate;
    /*I2C Address*/
    I2C_Address i2c_Add;
}USER_CONFIG_PARA;

/**
 * @brief UART Object Structure
 */
typedef struct
{
    /**
     * @brief Valid Sign
     */
    int8_t valid_sign;
    /**
     * @brief Header Buffer
     */
    BYTE headBuf[UART_CMD_HEADER_LEN];
    
    /**
     * @brief UART I2C Parameters
     */
    struct
    {
        BYTE uI2CChannel:1; // 0 or 1
        BYTE uI2CAddLen:1; // 0 or 1
        BYTE uI2CSpeed:2; // 0x00, 0x01,0x02,0x03
        BYTE uI2CPullState:1; // 0 or 1
        BYTE uI2CDatabufLen;
        uint16_t uI2CSlaveAdd;
        uint8_t uI2CRegSize; // 1 for 8 bit, 2 for 16 bit
        uint16_t uI2CRegAdd;
        BYTE uI2CHandle:1;
    }uarti2cparam;
    /**
     * @brief UART SPI Parameters
     */
    struct
    {
        BYTE uSPIHandle:1;
        BYTE uSPIChannel:1;
        BYTE uSPIMode:2;
        uint8_t uSPI_CSType;
        WORD uSPI_Speed;
        BYTE uSPI_CS_Polarity:1;
    }uartspiparam;

    struct
    {
        BYTE uUARTHandle:1;
        BYTE uUARTChannel:1;
    }uartuartparam;
}UART_OBJ;

#endif
