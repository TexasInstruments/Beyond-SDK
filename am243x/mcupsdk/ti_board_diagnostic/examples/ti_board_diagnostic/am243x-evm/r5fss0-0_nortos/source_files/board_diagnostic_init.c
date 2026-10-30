/**
 * @file Board_Diagnostic_Init.c
 * @brief Board Diagnostic system initialization implementation
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements the system initialization routine for the TI AM2432 
 * Board Diagnostic project, including GPIO controller setup and CAN driver management.
 */

#include "../header_files/common_extern.h"


/**
 * @brief Initialize the Board Diagnostic system
 * 
 * This function performs essential system initialization tasks:
 * - Sets up the external pin controller base address
 * - Ensures CAN driver is properly closed for clean startup
 * 
 * @note This should be called during system startup before other peripherals
 */
void Board_Diagnostic_Init(void)
{    
    /* Disable CAN Driver to ensure clean startup state */
    CANFD_close(gCanfdHandle[CONFIG_MCAN0]);
}
