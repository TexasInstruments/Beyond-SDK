/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef PSRAM_OSPI_H_
#define PSRAM_OSPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/ospi.h>
#include <board/psram.h>
typedef struct {

    OSPI_Handle ospiHandle;
    uint8_t dacEnable;
    uint8_t phyEnable;
    uint8_t numAddrBytes;
    uint8_t badBlockCheck;
    uint8_t syncModeEnable; // parameter to set mode type. 1 = Sync read and write; 0x00 = linear burst read and write
} Psram_OspiObject;

/* PSRAM specific externs */
extern Psram_Fxns gPsramOspiFxns;

int32_t Psram_ospiOpen(Psram_Config *config);
int32_t Psram_ospiRead(Psram_Config *config, uint32_t offset, uint8_t *pbuf, uint32_t len);
int32_t Psram_ospiWrite(Psram_Config *config, uint32_t offset,uint8_t *pbuf, uint32_t len);
void Psram_ospiClose(Psram_Config *config);
int32_t Psram_ospiReadCmd(Psram_Config *config, uint8_t cmd, uint32_t ReadAddr, uint8_t *rxBuf, uint32_t rxLen);
int32_t Psram_ospiReadId(Psram_Config *config, uint32_t *manufacturerId, uint32_t *deviceId);
int32_t Psram_ospiReset(Psram_Config *config);
int32_t Psram_ospiWriteCmd(Psram_Config *config, uint8_t cmd, uint32_t WritedAddr, uint8_t *txBuf, uint32_t txLen);

#ifdef __cplusplus
}
#endif

#endif /* PSRAM_OSPI_H_ */
