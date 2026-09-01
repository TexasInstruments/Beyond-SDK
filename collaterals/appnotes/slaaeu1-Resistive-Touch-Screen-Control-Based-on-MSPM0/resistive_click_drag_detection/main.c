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

/**
 * the threshold can change based on the sensitivity of the resistive panel and 
 * the use case. For a more responsive feel, the threshold can be reduced, but 
 * it may lead to some false positives for long touches. For a more deliberate 
 * long touch detection, the threshold can be increased, but it may lead to some 
 * false negatives for long touches and drag.
 * Belos macro - Threshold for differentiation between click and long press in ms
 */
#define LONG_TOUCH_THRESHOLD_MS 80

/* Sampling rate in ms for cases when there is no touch detected */
#define INACTIVE_SAMPLE_DELAY 200

/**
 * Needs to be modified based on which peripheral is being used to transfer the data. 
 * It is currently configured for UART at 9600 baud rate. UART takes 4 ms for a UART 
 * send command - can be verified by toggling a GPIO pin and measuring the time 
 * difference. If using a different peripheral, this delay needs to be adjusted to 
 * ensure no data is lost.
 * Belos macro - Delay between consecutive samples while touch is still detected in ms.
 */
#define TOUCH_RELEASE_SAMPLING_RATE 4
 /**
 * delay for voltage to be steady after setting pins in ms - calculated based on RC 
 * time constant of the touch screen circuit (can be adjusted based on empirical 
 * testing for better performance) 
 */
#define SETTLE_DELAY 3.125

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    uint32_t touchStartTimerCount = 0, currentTimerCount = 0, elapsedCycles = 0, 
        elapsedMs = 0, touchEndTimerCount = 0, totalElapsedCycles = 0;
    uint16_t x,y, sampleX, sampleY;

    /* Start Timer G for accurate timing */
    DL_TimerG_startCounter(TIMER_G_INST);

    uartSendString("DEBUG: Touch detection started\r\n", 32);

    while (1)
    {
        if(touchIODetection())
        {

            /* Small settle delay */
            delay_cycles(SETTLE_DELAY*32000);
            x = readTouchX();
            y = readTouchY();

            if (x >= X_MIN && y >= Y_MIN && x <= X_MAX && y <= Y_MAX)
            {
                /* Measure touch duration using Timer G - accurate wall-clock time */
                touchStartTimerCount = DL_TimerG_getTimerCount(TIMER_G_INST);
                uartSendString("\r\nTOUCH DETECTED! ", 18);

                /* Print both values together */
                uartSendString("X=", 2);
                uartSend((double)x);
                uartSendString("  Y=", 4);
                uartSend((double)y);

            }

            while (touchIODetection())
            {
                /* Get current timer count (counts down from ~4.3 billion) */
                currentTimerCount = DL_TimerG_getTimerCount(TIMER_G_INST);

                /* Calculate elapsed cycles (timer counts down, so subtract) */
                elapsedCycles = touchStartTimerCount - currentTimerCount;

                /**
                 * Convert cycles to milliseconds
                 * Clock = LFCLK (32768 Hz) / (254+1 prescale) = ~128.5 Hz
                 * 1 count = 7.776 ms
                 * elapsedMs = (elapsedCycles * 7776) / 1000;
                 * Only sample and print if already in long touch territory
                 */
                if (elapsedMs >= LONG_TOUCH_THRESHOLD_MS)
                {
                    
                    sampleX = readTouchX();
                    sampleY = readTouchY();

                    if (sampleX >= X_MIN && sampleY >= Y_MIN && 
                        sampleX <= X_MAX && sampleY <= Y_MAX)
                    {
                        /* Print in real-time for long touch: X,Y pairs */
                        uartSendString(" (", 2);
                        uartSend((double)sampleX);
                        uartSendString(",", 1);
                        uartSend((double)sampleY);
                        uartSendString(")", 1);
                    }
                }
                /* Small delay between reads - (A ms / 1000)s * 32 MHz = B cycles */
                delay_cycles(TOUCH_RELEASE_SAMPLING_RATE*32000);
            }

            /* Calculate total elapsed time using Timer G */
            touchEndTimerCount = DL_TimerG_getTimerCount(TIMER_G_INST);
            totalElapsedCycles = touchStartTimerCount - touchEndTimerCount;
            elapsedMs = (totalElapsedCycles * 7776) / 1000;

            if (x >= X_MIN && y >= Y_MIN && x <= X_MAX && y <= Y_MAX)
            {
                uartSendString("  TIME=", 7);
                uartSend((double)elapsedMs);
                uartSendString("ms", 2);

                /* Determine if long touch */
                if (elapsedMs >= LONG_TOUCH_THRESHOLD_MS)
                {
                    uartSendString(" - LONG TOUCH\r\n", 15);
                }
                else
                {
                    uartSendString(" - CLICK\r\n", 10);
                }
            }
            /* While no touch, delay for sampling ADC - (A ms / 1000)s * 32 MHz = B cycles */
            delay_cycles(INACTIVE_SAMPLE_DELAY*32000);
        }
    }
}

