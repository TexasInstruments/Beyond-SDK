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

/*
 * Manual OSPI PSRAM configuration header.
 * Defines the OSPI_Attrs for PSRAM (CS1, 200 MHz, no DMA, no PHY).
 * Used to reconfigure the single OSPI0 controller at runtime by swapping
 * the attrs pointer in gOspiConfig[CONFIG_OSPI0] between flash and PSRAM.
 */

#ifndef OSPI_PSRAM_CONFIG_H_
#define OSPI_PSRAM_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/ospi.h>

/* PSRAM OSPI attributes: CS1, 200 MHz, no DMA, no PHY, DAC enabled */
extern OSPI_Attrs gOspiAttrs_Psram;

/*
 * gOspiConfig is defined in ti_drivers_config.c.
 * We declare it here so main.c can swap the attrs pointer.
 */
extern OSPI_Config gOspiConfig[];

#ifdef __cplusplus
}
#endif

#endif /* OSPI_PSRAM_CONFIG_H_ */
