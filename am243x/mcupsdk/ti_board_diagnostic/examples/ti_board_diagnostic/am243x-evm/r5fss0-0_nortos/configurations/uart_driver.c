#include <drivers/uart/v0/lld/uart_lld.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/sciclient.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "uart_driver.h"

/*
 * UART
 */
UARTLLD_Object gUart0Object[1];
UARTLLD_Handle gUart0HandleLld[1];


uint32_t gUart0DmaConfigNumLld = 1;
/*
 * UART
 */

/* UART init objects */

UARTLLD_InitObject gUart0InitObject[1] =
{
        {
        .inputClkFreq       = 48000000,
        .baudRate           = 115200,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readReturnMode     = UART_READ_RETURN_MODE_FULL,
        .hwFlowControl      = FALSE,
        .hwFlowControlThr   = UART_RXTRIGLVL_16,
        .transferMode       = UART_CONFIG_MODE_POLLED,
        .intrNum            = 210U,
        .intrPriority       = 4U,
        .operMode           = UART_OPER_MODE_16X,
        .rxTrigLvl          = UART_RXTRIGLVL_8,
        .txTrigLvl          = UART_TXTRIGLVL_32,
        .rxEvtNum           = 0U,
        .txEvtNum           = 0U,
        .clockP_get         = ClockP_getTicks,
        .clockP_usecToTick  = ClockP_usecToTicks,
        .writeCompleteCallbackFxn = NULL,
        .readCompleteCallbackFxn  = NULL,
        },
};

int32_t status = UART_STATUS_SUCCESS;
/* UART Driver handles */
uint32_t Drivers_uart0OpenLld(void)
{
    gUart0HandleLld[0]             = &gUart0Object[0];
    gUart0HandleLld[0]->state      = UART_STATE_RESET;
    gUart0HandleLld[0]->baseAddr   = CSL_UART0_BASE;
    gUart0HandleLld[0]->hUartInit = &gUart0InitObject[0];

    status = UART_lld_init(gUart0HandleLld[0]);

    if(status != UART_STATUS_SUCCESS)
    {
        return -1;
        DebugP_assert(status == UART_STATUS_SUCCESS);
    }
    return UART_STATUS_SUCCESS;
}

uint32_t Drivers_uart0CloseLld(void)
{
    int32_t status = UART_STATUS_SUCCESS;

    status = UART_lld_deInit(gUart0HandleLld[0]);

    DebugP_assert(status == UART_STATUS_SUCCESS);
    return 0;
}
