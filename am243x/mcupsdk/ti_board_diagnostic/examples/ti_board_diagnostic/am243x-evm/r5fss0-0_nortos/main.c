/**
 * @file main.c
 * @brief Main application entry point for TI AM2432 Texas Instruments Diagnostic project
 * @author Tushar Thakur
 * 
 * This file contains the main application initialization
 * for the Texas Instruments UART Diagnostics project. It initializes
 * all system modules, and run the UART task to get commands.
 */

#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "header_files/common_extern.h"
#include <drivers/pinmux.h>
#include "configurations/interrupt_driver.h"


/* Clock parameters for system timing */
ClockP_Params clockParam;

/* Thread Function Declarations */
void xTask_UART();

/**
 * @brief Main application entry point
 * @return Will never return.
 * 
 * Initializes all system modules, and starts the
 * delay time. This function performs complete system initialization
 * including SOC, board, drivers, and application-specific modules.
 */
int main(void)
{
    /* Initialize System on Chip (SOC) specific modules */
    System_init();
    
    /* Initialize board-specific hardware and pin configurations */
    Board_init();
    
    /* Open and initialize all TI drivers */
    Drivers_open();
    
    /* Close general purpose timer drivers */
    Drivers_GpTimerClose();
    
    /* Open board-specific drivers after basic driver initialization */
    Board_driversOpen();
    
    /* Initialize Board Diagnostics-specific hardware and protocols */
    Board_Diagnostic_Init();

    xTask_UART();

    return 0;
}
