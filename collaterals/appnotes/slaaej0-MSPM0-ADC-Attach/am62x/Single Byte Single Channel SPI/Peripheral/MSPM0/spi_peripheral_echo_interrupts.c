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

volatile uint8_t gRxData, gTxData;
volatile uint8_t gADCResult = 0;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(SPI_0_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);

    NVIC_EnableIRQ(ADC_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_ADC_INST);

    gTxData = 1;

    while (1) {
        gTxData = gADCResult;
        if(gTxData >= 175){
            DL_GPIO_clearPins(RGB_PORT, RGB_GREEN_PIN);
            DL_GPIO_setPins(RGB_PORT, RGB_RED_PIN);
        }
        else{
            DL_GPIO_clearPins(RGB_PORT, RGB_RED_PIN);
            DL_GPIO_setPins(RGB_PORT, RGB_GREEN_PIN);
        }
    }
}

void SPI_0_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
        case DL_SPI_IIDX_RX:
            gRxData = DL_SPI_receiveData8(SPI_0_INST);
            DL_SPI_transmitData8(SPI_0_INST, gTxData);

            /* Toggle the Blue LED after data reception */
            DL_GPIO_togglePins(RGB_PORT, RGB_BLUE_PIN);

            break;
        default:
            break;
    }
}

void ADC_INST_IRQHandler(void)
{
        switch (DL_ADC12_getPendingInterrupt(ADC_INST)) {
            case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
                gADCResult = DL_ADC12_getMemResult(ADC_INST, DL_ADC12_MEM_IDX_0);
                break;
            default:
                break;
        }
}
