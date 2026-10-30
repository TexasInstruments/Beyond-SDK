/**
 * @file xTask_Runner.c
 * @brief Task implementations for TI AM2432 Board Diagnostic project
 * @author Tushar Thakur
 * 
 * This file contains the implementation of tasks for the Board Diagnostic
 * UART COM Ports communication project. It includes the main application task
 * and UART COM Ports task with proper timing and interrupt handling.
 */

#include "header_files/common_extern.h"

extern void uart_echo();

void DelayTimerInit()
{
    gGpTimerHandle[3] = GPTIMER_open(3, &gGpTimerParams[3]);
       if (gGpTimerHandle[3] == NULL)
       {
          DebugP_logError("ERROR: GPTIMER_open(3) failed. Staying in INIT.\n");
          /* Remain in INIT or add a retry/backoff mechanism. */
       }

    /* Start GP Timer */
       GPTIMER_start(gGpTimerHandle[3]);
}


/**
 * @brief UART COM Ports task
 * 
 * This task handles UART COM Ports communication operations. It initializes
 * the UART COM Ports subsystem and runs the main UART communication loop.
 * The task deletes itself when UART operations are complete or on error.
 */
void xTask_UART()
{
    DelayTimerInit();
    /* Initialize and run UART COM Ports main communication loop */
    for(;;)
    {
        uart_echo();
        IRQ_Packet_Transfer();
    }
}
