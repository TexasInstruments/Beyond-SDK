/**************************************************************************************************
 * @file common_extern.h
 * @brief External declarations for common functions and variables used across the Board Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This header file contains external declarations for all global variables and functions
 * that are shared across multiple modules in the TI AM2432 Board Diagnostic project.
 **************************************************************************************************/

#ifndef COMMON_EXTERN_H
#define COMMON_EXTERN_H

/*--------------------------------- STANDARD LIBRARY INCLUDES -----------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

/*------------------------------------ TI DRIVER INCLUDES --------------------------------------*/
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/pinmux.h>

/*----------------------------------- PROJECT INCLUDES -----------------------------------------*/
#include "global.h"
#include "../configurations/uart_driver.h"
#include "../configurations/gpio_driver.h"
#include "../configurations/interrupt_driver.h"

/**************************************************************************************************
 *                              EXTERNAL VARIABLE DECLARATIONS
 **************************************************************************************************/

/*------------------------------- I2C/UART CONFIGURATION VARIABLES ------------------------------*/
/**
 * @brief I2C user configuration parameters array
 * Array containing configuration parameters for each I2C instance
 */
extern USER_CONFIG_PARA i2c_userConfig[CONFIG_I2C_LLD_NUM_INSTANCES];

/**
 * @brief UART user configuration parameters array
 * Array containing configuration parameters for each UART instance
 */
extern USER_CONFIG_PARA uart_userConfig[CONFIG_I2C_LLD_NUM_INSTANCES];

/*------------------------------------ UART GLOBAL VARIABLES -------------------------------------*/
/**
 * @brief Global UART objects structure
 * Contains all UART-related configuration and state information
 */
extern UART_OBJ gUARTCmdObjects;

/*----------------------------------- SYSTEM STATUS VARIABLES -----------------------------------*/
/**
 * @brief General timeout counter
 * Used for various timeout operations across the system
 */
extern uint32_t timeout;

/**
 * @brief Board Diagnostic system status
 * Current operational status of the Board Diagnostic system
 */
extern int32_t ti_status;

/*------------------------------------ INTERRUPT VARIABLES --------------------------------------*/
/**
 * @brief Interrupt event counters array
 * Counts events for each interrupt pin (MAX_INT_PIN pins supported)
 */
extern volatile int32_t _INT_EventCout[MAX_INT_PIN];


/*-------------------------------------- CAN VARIABLES -----------------------------------------*/
/**
 * @brief CAN read data buffer
 * Buffer for storing data received via CAN FD communication
 */
extern uint8_t gCANReadBuff[CANFD_READ_DATA_LEN];

/**
 * @brief CAN TX message ID range and data length
 * Parameters for CAN transmit message objects
 */
extern uint32_t startMsgID_Tx, endMsgID_Tx, dataLen_Tx;

/**
 * @brief CAN RX message ID range and data length
 * Parameters for CAN receive message objects
 */
extern uint32_t startMsgID_Rx, endMsgID_Rx, dataLen_Rx;

/**
 * @brief CAN transmit ISR flag
 * Flag indicating CAN transmit interrupt status
 */
extern volatile uint8_t gCANTxISRFlage;

/**
 * @brief CAN receive ISR flag
 * Flag indicating CAN receive interrupt status
 */
extern volatile uint8_t gCANRxISRFlage;

/*------------------------------------ VARIABLES --------------------------------------*/
/**
 * @brief TX semaphore timeout value
 * Timeout value for transmit semaphore operations.
 */
extern uint32_t xSemaphoreTXTimeOut;

/**
 * @brief RX semaphore timeout value
 * Timeout value for receive semaphore operations
 */
extern uint32_t xSemaphoreRXTimeOut;

/*----------------------------------- FUNCTION CODE VARIABLES -----------------------------------*/
/**
 * @brief Current function code enumeration
 * Indicates the active Board Diagnostic function being executed
 */
extern enum TI_FUNCTION_CODE gFun_Code;

/**************************************************************************************************
 *                              EXTERNAL FUNCTION DECLARATIONS
 **************************************************************************************************/

/*------------------------------------- DRIVER MANAGEMENT ----------------------------------------*/
/**
 * @brief Initialize and open all hardware drivers
 * Initializes all configured hardware drivers for the system
 */
extern void Drivers_open(void);

/**
 * @brief Close all hardware drivers
 * Properly shuts down and closes all hardware drivers
 */
extern void Drivers_close(void);

/*------------------------------------------ I2C FUNCTIONS --------------------------------------*/
/**
 * @brief Configure I2C channel with specified speed
 * @param ch I2C channel number
 * @param speed I2C communication speed
 * @return Configuration status (0 = success)
 */
extern int I2C_Config(I2C_Channel ch, I2C_Speed speed);

/**
 * @brief Set I2C device address
 * @param i2cAdd I2C device address
 * @param param Pointer to I2C extended parameters
 */
extern void I2C_DEVICE_ADDRESS_SET(uint16_t i2cAdd, uint8_t regSize, I2C_ExtendedParams* param);

/**
 * @brief Initialize I2C transaction buffer
 * @param paramT Pointer to I2C extended parameters
 * @param regAdd Register address
 * @param dataSize Data size for transaction
 */
extern void Init_I2C_Transaction_Buf(I2C_ExtendedParams* paramT, regAddress regAdd, BYTE dataSize);

/**
 * @brief Write single register via I2C
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param val Pointer to value to write
 * @return Operation status (0 = success)
 */
extern int I2C_RegWrite(UART_OBJ* uI2CConfigParam, BYTE* val);

/**
 * @brief Read single register via I2C
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param readByte Pointer to store read byte
 * @return Operation status (0 = success)
 */
extern int I2C_RegRead(UART_OBJ* uI2CConfigParam, BYTE* readByte);

/**
 * @brief Write multiple registers via I2C
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param start_regAdd Starting register address
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int I2C_MultiRegWrite(UART_OBJ* uI2CConfigParam, regAddress start_regAdd, BYTE nBytes, BYTE *data);

/**
 * @brief Read multiple registers via I2C
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param start_regAdd Starting register address
 * @param nBytes Number of bytes to read
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int I2C_MultiRegRead(UART_OBJ* uI2CConfigParam, regAddress start_regAdd, BYTE nBytes, BYTE *data);

/**
 * @brief Raw I2C write operation
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int I2C_RawWrite(UART_OBJ* uI2CConfigParam, BYTE nBytes, BYTE *data);

/**
 * @brief Raw I2C read operation
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param nBytes Number of bytes to read
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int I2C_RawRead(UART_OBJ* uI2CConfigParam, BYTE nBytes, BYTE *data);

/**
 * @brief Combined I2C write and read operation
 * @param uI2CConfigParam UART I2C configuration parameters
 * @param nWriteBytes Number of bytes to write
 * @param writeData Pointer to write data buffer
 * @param nReadBytes Number of bytes to read
 * @param readData Pointer to read data buffer
 * @return Operation status (0 = success)
 */
extern int I2C_BlockWriteRead(UART_OBJ* uI2CConfigParam, BYTE nWriteBytes, BYTE *writeData, BYTE nReadBytes, BYTE* readData);

/**
 * @brief Test function for UART echo
 */
extern void uart_echo(void);

/**
 * @brief Configure UART channel
 * @param ch UART channel number
 * @param baud UART baudrate
 * @param parity UART parity
 * @param dataLen UART data length
 * @param stopBITLen UART stop bit length
 * @return Configuration status (0 = success)
 */
extern int UART_CONFIG(UART_CH ch, UART_Baudrate baud, BYTE parity, BYTE dataLen, BYTE stopBITLen);

/**
 * @brief Write data to UART
 * @param uUARTConfigParam UART UART configuration parameters
 * @param nBytes Number of bytes to write
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int UART_Write(UART_OBJ* uUARTConfigParam, BYTE nBytes, const void *data);

/**
 * @brief Read data from UART
 * @param uUARTConfigParam UART UART configuration parameters
 * @param nBytes Number of bytes to read
 * @param data Pointer to data buffer
 * @return Operation status (0 = success)
 */
extern int UART_Read(UART_OBJ* uUARTConfigParam, BYTE nBytes, void *data);

/**
 * @brief Disable UART receiver
 * @param channel UART channel number
 * @return Operation status (0 = success)
 */
extern int UART_DISABLE_RECEIVER(uint8_t channel);

/**
 * @brief Get UART receive count
 * @param channel UART channel number
 * @return Number of bytes in receive buffer
 */
extern int UART_GET_RX_CNT(uint8_t channel);

/**
 * @brief Set UART mode
 * @param channel UART channel number
 * @param mode UART mode to set
 * @return Operation status (0 = success)
 */
extern int UART_SET_MODE(uint8_t channel, uint32_t mode);

/*------------------------------------------ SPI FUNCTIONS --------------------------------------*/
/**
 * @brief Configure SPI channel
 * @param ch SPI channel number
 * @param nCS Number of chip select pins
 * @param mode SPI frame format
 * @param clkFreq SPI clock frequency
 * @param cs_Type Chip select type
 * @param cspolarity Chip select polarity
 * @return Configuration status (0 = success)
 */
extern int32_t SPI_Config(SPI_CH ch, BYTE nCS, SPI_FrameFormat mode, SPI_Clock clkFreq, bool cs_Type, SPI_CSPolarity cspolarity);

/**
 * @brief SPI read/write operation
 * @param uSPIConfigParam UART SPI configuration parameters
 * @param txData Pointer to transmit data buffer
 * @param rxData Pointer to receive data buffer
 * @param byteCount Number of bytes to transfer
 * @return Operation status (0 = success)
 */
extern int32_t SPI_ReadWrite(UART_OBJ* uSPIConfigParam, void* txData, void* rxData, WORD byteCount);

/**
 * @brief UART task
 */
extern void xTask_UART();

/**
 * @brief Parse UART command
 * @param UARTPayload Pointer to UART payload data
 * @return Parse status (0 = success)
 */
extern int8_t gParseUARTCMD(BYTE* UARTPayload);

/**
 * @brief Open UART0 LLD driver
 * @return Driver handle or error code
 */
extern uint32_t Drivers_uart0OpenLld(void);

/*------------------------------------------ GPIO FUNCTIONS -------------------------------------*/
/**
 * @brief Write GPIO control register
 * @param gpioPayload Pointer to GPIO control payload
 * @return Operation status (0 = success)
 */
extern uint8_t GPIO_WriteControl(uint8_t* gpioPayload);

/**
 * @brief Write GPIO state
 * @param gpioPayload Pointer to GPIO state payload
 * @return Operation status (0 = success)
 */
extern uint8_t GPIO_WriteState(uint8_t* gpioPayload);

/**
 * @brief Read GPIO state
 * @param nReadBytes Number of bytes to read
 * @param readBuff Pointer to read buffer
 * @return Operation status (0 = success)
 */
extern uint8_t GPIO_ReadState(uint8_t nReadBytes, uint8_t* readBuff);

/*--------------------------------------- INTERRUPT FUNCTIONS -----------------------------------*/
/**
 * @brief Transfer interrupt packet
 * Handles interrupt packet transmission
 */
extern void IRQ_Packet_Transfer(void);

/**
 * @brief Send ISR packets
 * @param ch Channel number
 * @param counts Number of packets to send
 */
extern void SendISRPackets(uint8_t ch, uint32_t counts);

/*------------------------------------------ PWM FUNCTIONS --------------------------------------*/
/**
 * @brief Enable PWM channel
 * @param pwmChannel PWM channel number
 * @param pwmFreq PWM frequency
 * @param pwmDutyCycle PWM duty cycle
 * @return Operation status (0 = success)
 */
extern int32_t PWM_ENABLE(uint8_t pwmChannel, uint32_t pwmFreq, uint32_t pwmDutyCycle);

/**
 * @brief Disable PWM channel
 * @param _pwmChannel PWM channel number
 * @return Operation status (0 = success)
 */
extern int32_t PWM_DISABLE(int8_t _pwmChannel);

/*----------------------------------------- CANFD FUNCTIONS ------------------------------------*/
/**
 * @brief Configure CAN FD
 * @param configParamsPayload Pointer to configuration parameters
 * @return Configuration status (0 = success)
 */
extern int32_t CANFD_Configure(uint32_t* configParamsPayload);

/**
 * @brief Create CAN FD transmit message object
 * @param id Message ID type
 * @param startMsgID Start message ID
 * @param endMsgID End message ID
 * @param dataLen Data length
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_Transmit_MsgObjectCreate(CANFD_MCANXidType id, uint32_t startMsgID, uint32_t endMsgID, uint32_t dataLen);

/**
 * @brief Create CAN FD receive message object
 * @param id Message ID type
 * @param startMsgID Start message ID
 * @param endMsgID End message ID
 * @param dataLen Data length
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_Recv_MsgObjectCreate(CANFD_MCANXidType id, uint32_t startMsgID, uint32_t endMsgID, uint32_t dataLen);

/**
 * @brief Receive data via CAN FD
 * @param rxdata Pointer to receive data buffer
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_ReceiveData(uint8_t *rxdata);

/**
 * @brief Transmit data via CAN FD
 * @param txdata Pointer to transmit data buffer
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_TransmitData(const uint8_t *txdata);

/**
 * @brief Delete CAN FD transmit object
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_deleteTXObject(void);

/**
 * @brief Delete CAN FD receive object
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_deleteRXObject(void);

/**
 * @brief Get CAN FD transmit message handle
 * @return Message object handle
 */
extern CANFD_MsgObjHandle CANFD_getTX_Msg_Handle(void);

/**
 * @brief Get CAN FD receive message handle
 * @return Message object handle
 */
extern CANFD_MsgObjHandle CANFD_getRX_Msg_Handle(void);

/**
 * @brief Validate CAN FD data length
 * @param len Data length to validate
 * @return Validation status (0 = success)
 */
extern int32_t CANFD_ValidateDataLen(uint32_t len);

/**
 * @brief Get CAN FD default parameters
 * @param defaultParams Pointer to store default parameters
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_GetDefaultParams(uint8_t *defaultParams);

/**
 * @brief Get CAN FD failure reason
 * @return Failure reason code
 */
extern int32_t CANFD_GetFailureReason(void);

/**
 * @brief Convert uint8 array to uint32 array
 * @param inBuff Input buffer (uint8)
 * @param outBuff Output buffer (uint32)
 * @param len Input buffer length
 * @param outCapacityWords Output buffer capacity in words
 * @return Conversion status (0 = success)
 */
extern int32_t u8_to_u32_convert(const uint8_t *inBuff, uint32_t *outBuff, size_t len, size_t outCapacityWords);

/**
 * @brief Convert uint32 to uint8 array (big endian)
 * @param data 32-bit data to convert
 * @param buffer Output buffer (uint8)
 */
extern void convert32To8_BE_ptr(uint32_t data, uint8_t *buffer);

/**
 * @brief Close CAN FD
 * @return Operation status (0 = success)
 */
extern int32_t CANFD_Close(void);

/**
 * @brief Performs DDR Memory testing by writing and reading known patterns
 * @param offset DDR memory offset to read/write from.
 * @param blockSize Size of bytes in multiple of 8(64-bits).
 * @return Operation status (0 = success)
 */
extern int32_t DDR_Memtester(uint32_t *offset, uint32_t blockSize);

/**
 * @brief Perform read write operation on DDR memory.
 * @param nBytes number of bytes to read/write.
 * @param writeData Data to be written on DDR memory.
 * @param readData Data to be read from DDR memory.
 * @return Operation status (0 = success)
 */
extern int32_t DDR_ReadWrite(uint32_t *offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData);

/**
 * @brief Perform read write operation on EMMC memory.
 * @param offset EMMC memory offset to read/write from.
 * @param nBytes number of bytes to read/write.
 * @param writeData Data to be written on EMMC memory.
 * @param readData Data to be read from EMMC memory.
 * @return Operation status (0 = success)
 */
extern int32_t EMMC_ReadWrite(uint32_t offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData);

/**
 * @brief Perform read write operation on OSPI Flash.
 * @param offset OSPI Flash offset to read/write from.
 * @param nBytes number of bytes to read/write.
 * @param writeData Data to be written on OSPI Flash.
 * @param readData Data to be read from OSPI Flash.
 * @return Operation status (0 = success)
 */
extern int32_t OSPI_ReadWrite(uint32_t offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData);

/*--------------------------------------- SYSTEM FUNCTIONS --------------------------------------*/
/**
 * @brief Initialize Board Diagnostic system
 * Initializes all system components and peripherals
 */
extern void Board_Diagnostic_Init(void);

#endif
