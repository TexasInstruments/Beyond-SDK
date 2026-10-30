#include <drivers/uart.h>
#include "../header_files/global.h"
#include "ti_drivers_open_close.h"
#include "kernel/dpl/ClockP.h"

UART_OBJ gUARTCmdObjects;
extern int8_t gParseUARTCMD(BYTE* uartPayload);

#define UART_CMD_FUN_CODE_ID              6U
#define UART_CMD_CODE_ID            UART_CMD_FUN_CODE_ID + 1
#define UART_DATA_LEN_ID            UART_CMD_CODE_ID + 1
#define UART_CMD_LEN                UART_DATA_LEN_ID + 1

/**
 * @brief Initialize UART object structure
 * @param uartParm Pointer to UART object to initialize
 *
 * Sets UART signature to invalid and clears header buffer.
 */
static void UARTObjectStructInit(UART_OBJ* uartParm)
{
    uartParm->valid_sign = INVALID;
    memset(uartParm->headBuf,0,UART_CMD_HEADER_LEN);
}


void uart_echo()
{
    UART_Transaction trans;
    UART_Transaction_init(&trans);

    /* Initialize UART object structure with default values */
    UARTObjectStructInit(&gUARTCmdObjects);

    uint8_t *buffer = (uint8_t*)malloc(sizeof(uint8_t) * 255);

    /* Read string */
    uint32_t transferOK;
    trans.buf   = &buffer[0U];
    trans.count = UART_CMD_LEN;
    transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);

    if(transferOK != 0)
    {
        DebugP_logError("Error while reading...\r\n");
        free(buffer);
        return;
    }


   /* Read string */
   if(buffer[UART_DATA_LEN_ID] > 0)
   {
      trans.buf   = &buffer[UART_CMD_LEN];
      trans.count = buffer[UART_DATA_LEN_ID];
      transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);

      if(transferOK != 0)
      {
          DebugP_logError("Error with Data Length...\r\n");
          free(buffer);
          return;
      }
   }

//   memcpy(gUARTCmdObjects.headBuf, buffer, UART_CMD_LEN);
   gParseUARTCMD(buffer);
   ClockP_usleep(100);
   free(buffer);
   return;

}
