/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "resistive_detection.h"


uint8_t readTouchX(void)
{
    touchIOReset();

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_XP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_XN_IOMUX);
    DL_GPIO_setPins(GPIO_GRP_0_PORT,  GPIO_GRP_0_XP_PIN);
    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_XN_PIN);
    DL_GPIO_enableOutput(GPIO_GRP_0_PORT, GPIO_GRP_0_XP_PIN |GPIO_GRP_0_XN_PIN);

    delay_cycles(10000);//Wait voltage to be steady
    return ADC_sample(Y_ADC_CHANNEL);
}

uint8_t readTouchY(void)
{
    touchIOReset();

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YN_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YP2_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YN2_IOMUX);
    DL_GPIO_setPins(GPIO_GRP_0_PORT,  GPIO_GRP_0_YP_PIN |GPIO_GRP_0_YP2_PIN);
    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_YN_PIN | GPIO_GRP_0_YN2_PIN);
    DL_GPIO_enableOutput(GPIO_GRP_0_PORT, GPIO_GRP_0_YP_PIN |GPIO_GRP_0_YN_PIN | GPIO_GRP_0_YP2_PIN |GPIO_GRP_0_YN2_PIN);

    delay_cycles(10000);//Wait voltage to be steady
    return ADC_sample(X_ADC_CHANNEL);
}

bool touchIODetection(void)
{
    uint8_t adcValue;
//    touchIOReset();
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_XN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_XP_IOMUX);
    //Set Yp and Yn to high
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YP_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YN_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YP2_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRP_0_YN2_IOMUX);
    DL_GPIO_setPins(GPIO_GRP_0_PORT,  GPIO_GRP_0_YP_PIN | GPIO_GRP_0_YP2_PIN| GPIO_GRP_0_YN_PIN | GPIO_GRP_0_YN2_PIN);
    DL_GPIO_enableOutput(GPIO_GRP_0_PORT,  GPIO_GRP_0_YP_PIN | GPIO_GRP_0_YP2_PIN| GPIO_GRP_0_YN_PIN | GPIO_GRP_0_YN2_PIN);

    delay_cycles(10000);//Wait voltage to be steady
    adcValue= ADC_sample(X_ADC_CHANNEL);


    if (adcValue>TOUCH_DETECTION_THD)
    {
      return true;
    }
    else
    {
        return false;
    }
}


void touchIOReset(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_XP_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_XN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_YP_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_YN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_YP2_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_GRP_0_YN2_IOMUX);
}

volatile bool gCheckADC;
uint8_t ADC_sample(uint32_t adc_channel)
{
    gCheckADC = false;
    DL_ADC12_disableConversions(ADC12_0_INST);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        adc_channel, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_ENABLED,
         DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    DL_ADC12_enableConversions(ADC12_0_INST);
    DL_ADC12_startConversion(ADC12_0_INST);
    while (false == gCheckADC) {
          __WFE();
      }

    return DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}



