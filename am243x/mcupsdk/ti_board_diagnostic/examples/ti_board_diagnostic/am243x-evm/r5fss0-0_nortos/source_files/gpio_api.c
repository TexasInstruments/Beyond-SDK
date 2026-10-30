/**
 * @file gpio_api.c
 * @brief GPIO API implementation for TI AM2432 Omnilink project
 * @author TI Omnilink Team
 * @version 1.0
 * @date 2026
 * 
 * This file implements GPIO control APIs including pin configuration,
 * state writing, and state reading operations for the Omnilink system.
 */

#include "../header_files/common_extern.h"
#include "../header_files/uart_cmd_header.h"

/**
 * @brief Configure GPIO control registers for all pins
 * @param gpioPayload Pointer to array containing GPIO control values
 * @return Number of pins successfully configured
 */
uint8_t GPIO_WriteControl(uint8_t* gpioPayload)
{
    uint8_t pinCnt; 
    uint8_t pinStatus = 0;
    
    /* Iterate through all GPIO pins and configure control registers */
    for (pinCnt = 0; pinCnt < GPIO_MAX_INDEX; pinCnt++)
    {
        pinStatus += GPIO_Set(pinCnt, gpioPayload[pinCnt]);
    }
    return pinStatus;
}

/**
 * @brief Write state values to all GPIO pins
 * @param gpioPayload Pointer to array containing GPIO state values
 * @return Number of pins successfully written
 */
uint8_t GPIO_WriteState(uint8_t* gpioPayload)
{
    uint8_t pinCnt; 
    uint8_t pinStatus = 0;
    
    /* Iterate through all GPIO pins and write their states */
    for (pinCnt = 0; pinCnt < GPIO_MAX_INDEX; pinCnt++)
    {
        pinStatus += GPIO_Write(pinCnt, gpioPayload[pinCnt]);
    }
    return pinStatus;    
}

/**
 * @brief Read boolean values from specified GPIO pins
 * @param nReadBytes Number of GPIO pins to read
 * @param readBuff Pointer to buffer to store read values
 * @return Number of pins successfully read
 */
uint8_t GPIO_ReadState(uint8_t nReadBytes, uint8_t* readBuff)
{
    uint8_t pinCnt;
    
    /* Read state from specified number of GPIO pins */
    for (pinCnt = 0; pinCnt < nReadBytes; pinCnt++)
    {
        readBuff[pinCnt] = GPIO_Read(pinCnt);
    }
    
    /* Return number of bytes read */
    return pinCnt;    
}
