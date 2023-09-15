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
#define NUM_CHANNELS (2U)
const uint8_t CHANNEL7 = 0;
const uint8_t CHANNEL8 = 1;

volatile uint8_t cmd = 0;
volatile uint8_t gTxData[NUM_CHANNELS] = {0, 0};
volatile uint8_t gADCResult[NUM_CHANNELS] = {0, 0};

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(SPI_0_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);

    NVIC_EnableIRQ(ADC_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_ADC_INST);

    while (1) {
        for(uint8_t i=0; i<NUM_CHANNELS; i++){
            gTxData[i] = gADCResult[i];
        }
    }
}

void SPI_0_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
        case DL_SPI_IIDX_RX:
            cmd = DL_SPI_receiveData8(SPI_0_INST);
            switch(cmd){
                case CHANNEL7:
                    DL_SPI_transmitData8(SPI_0_INST, gTxData[CHANNEL7]);
                    break;
                case CHANNEL8:
                    DL_SPI_transmitData8(SPI_0_INST, gTxData[CHANNEL8]);
                    break;
                default:
                    break;
            }
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
                break;
            case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
                gADCResult[CHANNEL8] = DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_1);
                break;
            default:
                break;
        }
}
