/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#define SPI_PACKET_SIZE (2U)
#define NUM_CHANNELS (2U)
const uint8_t CHANNEL7 = 0;
const uint8_t CHANNEL8 = 1;

uint8_t gTxPacket[NUM_CHANNELS][SPI_PACKET_SIZE];
uint8_t gRxPacket[SPI_PACKET_SIZE] = {0, 0};

volatile bool gDMARXDataTransferred, gReceiveReady;
volatile uint16_t gADCResult[NUM_CHANNELS] = {0, 0};


void SPI_receive(void);

int main(void)
{
    SYSCFG_DL_init();
    DL_SYSCTL_disableSleepOnExit();
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);

    NVIC_EnableIRQ(ADC_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_ADC_INST);

    gDMARXDataTransferred = false;
    gReceiveReady         = true;

    while (1) {
        if (gReceiveReady == true) {
            /*
             * Fill FIFO with data.
             * Note that transactions are initiated by the Controller, so this function
             * only fills the buffer and the Peripheral device will send this data when
             * requested by the Controller.
             */
            switch(gRxPacket[0]){
                case CHANNEL7:
                    DL_SPI_fillTXFIFO8(SPI_0_INST, &gTxPacket[CHANNEL7][0], SPI_PACKET_SIZE);
                    break;
                case CHANNEL8:
                    DL_SPI_fillTXFIFO8(SPI_0_INST, &gTxPacket[CHANNEL8][0], SPI_PACKET_SIZE);
                    break;
                default:
                    break;
            }
            gReceiveReady = false;
            SPI_receive();
        } else {
            __WFI();
        }
    }
}

void SPI_receive(void)
{
    /*
     * Configure DMA source, destination and size from RXDATA to gRxPacket.
     * The DMA transfer will start when the RX interrupt is set, which happens
     * when the device receives data.
     */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&SPI_0_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gRxPacket[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, SPI_PACKET_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    /*
     * Wait in SLEEP mode until data is received and DMA interrupt
     * is triggered
     */
    while (false == gDMARXDataTransferred) {
        __WFE();
    }
    gDMARXDataTransferred = false;
    gReceiveReady         = true;
}

void SPI_0_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
        case DL_SPI_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to gRxPacket*/
            gDMARXDataTransferred = true;
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            break;

        default:
            break;
    }
}

void ADC_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gADCResult[CHANNEL7] = DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_0);
            gTxPacket[CHANNEL7][0] = (gADCResult[CHANNEL7] & 0xFF00)>>8;
            gTxPacket[CHANNEL7][1] = (gADCResult[CHANNEL7] & 0x00FF);
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            gADCResult[CHANNEL8] = DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_1);
            gTxPacket[CHANNEL8][0] = (gADCResult[CHANNEL8] & 0xFF00)>>8;
            gTxPacket[CHANNEL8][1] = (gADCResult[CHANNEL8] & 0x00FF);
            break;
        default:
            break;
    }
}
