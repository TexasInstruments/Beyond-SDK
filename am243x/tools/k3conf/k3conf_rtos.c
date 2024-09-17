/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <k3conf/include/k3conf.h>


/*
 * This is an k3conf tool project provided for r5fss0-0 cores present in the device.
 * User can use this project to check the device info and status.
 *
 * This application does driver and board init and prints the info about the device.
 * In case of the main core, the print is redirected to the UART console.
 * The Application will not print anything on the console if used with interrupts.
 */

#define APP_UART_RECEIVE_BUFSIZE    100U
#define NEWLINE_CHAR                13U
#define BACKSPACE_CHAR              8U

void k3conf_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    char str[APP_UART_RECEIVE_BUFSIZE+1];
    char **argv = (char **)malloc(sizeof(char *) * 5);
    int32_t transferOK;
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    while(1)
    {
        char ch = 0;
        uint8_t itr = 0;

        /* Ask user to input query */
        DebugP_log("~#: ");
        /* Stop taking characters from UART console if a newline character is detected or buffer size is full */
        while((ch != NEWLINE_CHAR) && (itr < APP_UART_RECEIVE_BUFSIZE))
        {
            trans.buf   = &ch;
            trans.count = 1;
            transferOK = UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
            DebugP_assert((SystemP_SUCCESS == (transferOK)) && (UART_TRANSFER_STATUS_SUCCESS == trans.status));
            UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
            if(ch != BACKSPACE_CHAR)
            {
                str[itr++] = ch;
            }
            else if(itr > 0)
            {
                itr--;
            }
        }
        str[itr-1] = 0;
        DebugP_log("~#: %s\r\n", str);

        uint8_t start = 0;
        int argc = 0;
        uint8_t len;

        /* calculate length of input string */
        for(len = 0; str[len]; len++);

        /* create an array of strings from input string */
        for(uint8_t i=0; i<=len; i++)
        {
            if(str[i] == ' ' || str[i] == '\n' || str[i] == 0)
            {
                /* split string by spaces, newline and termination character */
                if((i - start) != 0)
                {
                    /* if split string's length is > 0 */
                    argv[argc] = (char*)malloc(sizeof(char)*20);
                    memcpy(argv[argc], &str[start], i-start);
                    argv[argc][i-start] = 0;
                    argc++;
                    start = i+1;
                }
                else
                {
                    /* if string is empty, or just contains spaces/newline/termination */
                    start = i+1;
                }
            }
        }
        show_main(argc, argv);
        for(int itr = 0; itr < argc; itr++)
        {
            free(argv[itr]);
            argv[itr] = NULL;
        }
    }

    Board_driversClose();
    Drivers_close();
}
