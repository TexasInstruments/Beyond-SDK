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

#include "UART.h"


void doubleToString(double num, char* str)
{
    int whole = (int)num;
    double fraction = num - whole;

    int i = 0;
    if (whole == 0)
    {
         str[i++] = '0';
     }
    else {
    while (whole != 0)
    {
        str[i++] = '0' + whole % 10;
        whole = whole / 10;
    }
    }
    int n = i - 1;
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[n];
        str[n] = temp;
        n--;
    }

    str[i++] = '.';

    for (int k = 0; k < 5; k++) {
        fraction *= 10;
        int digit = (int)fraction;
        str[i++] = '0' + digit;
        fraction -= digit;
    }
    return;
}

void uartSendString(char * buf, uint8_t Length)
{

    for(int i=0; i<Length; i++)
   {
       DL_UART_Main_transmitData(UART_0_INST, (uint8_t)buf[i]);
       while (DL_UART_isBusy(UART_0_INST))
       ;
   }

}

void uartSend(double num)
{
    char resultbuf[20];
    doubleToString(num,resultbuf);


    for(int i=0; i<FLOAT_DATA_LEN; i++)
   {
       DL_UART_Main_transmitData(UART_0_INST, resultbuf[i]);
       while (DL_UART_isBusy(UART_0_INST))
       ;
   }
    return;
}

void uartDataUpdate(uint8_t u8x, uint8_t u8y)
{
    uartSendString("X:", 2);
    uartSend((double)u8x); //Evaluate data
    uartSendString("  Y:", 4);
    uartSend((double)u8y); //Evaluate data
    uartSendString("\r\n", 2);
}

