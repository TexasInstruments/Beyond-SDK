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

#ifndef RESISTIVE_DETECTION_H_
#define RESISTIVE_DETECTION_H_

#define X_ADC_CHANNEL      DL_ADC12_INPUT_CHAN_9
#define Y_ADC_CHANNEL      DL_ADC12_INPUT_CHAN_8

/* scaling below macro from 8-bit to 12-bit resolution by: 250*16 */
#define TOUCH_DETECTION_THD     4000
 /**
 * delay for voltage to be steady after setting pins in ms - calculated based on RC 
 * time constant of the touch screen circuit (can be adjusted based on empirical 
 * testing for better performance) 
 */
#define SETTLE_DELAY 3.125

#include "ti_msp_dl_config.h"

uint16_t readTouchX(void);
uint16_t readTouchY(void);
bool touchIODetection(void);
void touchIOReset(void);
uint16_t ADC_sample(uint32_t adc_channel);

#endif /* RESISTIVE_DETECTION_H_ */
