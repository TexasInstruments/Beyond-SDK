/*
 * Copyright (c) 2026, Texas Instruments Incorporated
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
#include "UART.h"
#include "resistive_detection.h"

/* change the min and max values according to your touch screen's calibration data */
#define X_MIN 200
#define X_MAX 4090
#define Y_MIN 200
#define Y_MAX 3800
/* Sampling rate in ms for cases when there is no touch detected */
#define INACTIVE_SAMPLE_DELAY 200
/**
 * Delay for voltage to be steady after setting pins in ms - calculated based on 
 * RC time constant of the touch screen circuit (can be adjusted based on empirical 
 * testing for better performance)
 */
#define SETTLE_DELAY 3.125

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    uint16_t x,y;

    uartSendString("DEBUG: Touch detection started\r\n", 32);

    while (1)
    {
        if(touchIODetection())
        {
            /* Small settle delay */
            delay_cycles(SETTLE_DELAY*32000);
            x = readTouchX();
            y = readTouchY();
            
            /**
             * the following condition is added to check the validity of the touch 
             * coordinates. In some cases, when the screen is touched near the edges, 
             * due to noise, the coordinates can be out of bounds, which can lead to 
             * incorrect behavior in the subsequent code. This condition ensures that 
             * only valid touches within the calibrated range are processed further.
             */
            if (x >= X_MIN && y >= Y_MIN && x <= X_MAX && y <= Y_MAX)
            {
                /* Print the coordinates of the touch */
                uartSendString("\r\nTOUCH DETECTED! ", 18);
                uartSendString("X=", 2);
                uartSend((double)x);
                uartSendString("  Y=", 4);
                uartSend((double)y);
                uartSendString("\r\n", 2);
            }
            /* While no touch, delay for sampling ADC - (A ms / 1000)s * 32 MHz = B cycles */
            delay_cycles(INACTIVE_SAMPLE_DELAY*32000);
        }
    }
}

