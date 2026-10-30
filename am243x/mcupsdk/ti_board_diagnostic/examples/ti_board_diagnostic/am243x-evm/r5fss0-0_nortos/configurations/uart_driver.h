#include <stdint.h>
#include <drivers/uart.h>
#include <drivers/uart/v0/lld/dma/udma/uart_dma_udma.h>

extern UARTLLD_Object gUart0Object[1];
extern UARTLLD_Handle gUart0HandleLld[1];
extern UARTLLD_InitObject gUart0InitObject[1];
extern UART_UdmaChConfig gUart0DmaChCfg[1];

/*
 * UART Driver Advance Parameters - to be used only when Driver_open() and
 * Driver_close() is not used by the application
 */
/* UART Driver open/close - can be used by application when Driver_open() and
 * Driver_close() is not used directly and app wants to control the various driver
 * open/close sequences */
uint32_t Drivers_uart0OpenLld(void);
uint32_t Drivers_uart0CloseLld(void);