/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define APP_MCSPI_MSGSIZE                   (2U)
#define NUM_CHANNELS                        (2U)

#define CHANNEL7                            (0U)
#define CHANNEL8                            (1U)

uint8_t gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t gMcspiRxBuffer[APP_MCSPI_MSGSIZE];
uint8_t cmds[NUM_CHANNELS] = {CHANNEL7, CHANNEL8};
Bool isFirst = 1;



void *mcspi_loopback_main(void *args)
{
    int32_t             transferOK;
    MCSPI_Transaction   spiTransaction;

    Drivers_open();
    Board_driversOpen();

    while(1){


        for(uint8_t k=0; k<NUM_CHANNELS; k++){
            gMcspiTxBuffer[0] = cmds[k];
            gMcspiTxBuffer[1] = 0U;

            gMcspiRxBuffer[0] = 0U;
            gMcspiRxBuffer[1] = 0U;


            /* Initiate transfer */
            MCSPI_Transaction_init(&spiTransaction);
            spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
            spiTransaction.dataSize = 8;
            spiTransaction.csDisable = TRUE;
            spiTransaction.count    = APP_MCSPI_MSGSIZE / (spiTransaction.dataSize/8);
            spiTransaction.txBuf    = (void *)gMcspiTxBuffer;
            spiTransaction.rxBuf    = (void *)gMcspiRxBuffer;
            spiTransaction.args     = NULL;

            //The following function reads RXFIFO and transfers TXFIFO
            //We will be transferring command with id k, while
            //receiving data in response to command to with id (k-1)
            transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);

            if((SystemP_SUCCESS != transferOK) ||
               (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
            {
                DebugP_assert(FALSE); /* MCSPI transfer failed!! */
            }
            else
            {
                if(isFirst){
                    isFirst = 0;
                    continue;           //Since the first FIFO read contains garbage
                }

                uint8_t k_received = (k-1) % NUM_CHANNELS;
                uint16_t data = 0;
                data = gMcspiRxBuffer[0]<<8 | gMcspiRxBuffer[1];

                if(k_received==0){
                    DebugP_log("CommandID = %2u, Data = %4u\t\t", k_received, data);
                }
                else{
                    DebugP_log("CommandID = %2u, Data = %4u\r\n", k_received, data);
                }
            }
            ClockP_usleep(10000);
        }
    }
    Board_driversClose();
    Drivers_close();

    return NULL;
}
