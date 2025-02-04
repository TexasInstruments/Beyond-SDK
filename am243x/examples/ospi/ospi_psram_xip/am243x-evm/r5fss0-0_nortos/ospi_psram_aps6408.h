/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#ifndef OSPI_PSRAM_APS6408_H_
#define OSPI_PSRAM_APS6408_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/soc_config.h>

/**
 * This file contains description of the APS6408L OSPI PSRAM memory.
 * For additional details check APS6408L-OBMx datasheet (APM_PSRAM_OPI_Xccela-APS6408L-OBMx-v3.5b-PKG.pdf)
 * RAM SIZE is 0x800000   (64 MBits, 8 MBytes)
 */

/* Mode Register 0 */
#define OSPI_PSRAM_APS6408_MR0_ADDRESS            0x00000000U

#define OSPI_PSRAM_APS6408_MR0_DRIVE_STRENGTH     0x03U       /* Drive Strength                      */
#define OSPI_PSRAM_APS6408_MR0_READ_LATENCY_CODE  0x1CU       /* Read Latency Code                   */
#define OSPI_PSRAM_APS6408_MR0_RLC_3              0x00U       /* Read Latency Code 3                 */

#define OSPI_PSRAM_APS6408_MR0_LATENCY_TYPE       0x20U       /* Latency Type                        */

/* Mode Register 1 */
#define OSPI_PSRAM_APS6408_MR1_ADDRESS            0x00000001U

#define OSPI_PSRAM_APS6408_MR1_VENDOR_ID          0x1FU       /* Vendor Identifier                   */
#define OSPI_PSRAM_APS6408_MR1_VENDOR_ID_APM      0x0DU       /* Vendor Identifier APM               */

/* Mode Register 2 */
#define OSPI_PSRAM_APS6408_MR2_ADDRESS            0x00000002U
#define OSPI_PSRAM_APS6408_MR2_DEVICE_ID          0x18U       /* Device Identifier                   */
#define OSPI_PSRAM_APS6408_MR2_DEVID_GEN_3        0x02U       /* Device Identifier Generation 3      */

/* Mode Register 4 */
#define OSPI_PSRAM_APS6408_MR4_ADDRESS            0x00000004U
#define OSPI_PSRAM_APS6408_MR4_WLC_3              0x00U       /* Write Latency Code 3                */

/* Mode Register 6 */
#define OSPI_PSRAM_APS6408_MR6_ADDRESS            0x00000006U

/* Mode Register 8 */
#define OSPI_PSRAM_APS6408_MR8_ADDRESS            0x00000008U
#define OSPI_PSRAM_APS6408_MR8_BT                 0x04U       /* Burst Type                          */

/* Read Operations */
#define OSPI_PSRAM_APS6408_READ_CMD               0x00        /* Synchronous Read                    */
#define OSPI_PSRAM_APS6408_READ_LINEAR_BURST_CMD  0x20        /* Linear Burst Read                   */
#define OSPI_PSRAM_APS6408_READ_HYBRID_BURST_CMD  0x3F        /* Hybrid Burst Read                   */

/* Write Operations */
#define OSPI_PSRAM_APS6408_WRITE_CMD              0x80U       /* Synchronous Write                   */
#define OSPI_PSRAM_APS6408_WRITE_LINEAR_BURST_CMD 0xA0U       /* Linear Burst Write                  */

/* Reset Operations */
#define OSPI_PSRAM_APS6408_RESET_CMD              0xFFU       /* Global Reset                        */

/* Register Operations */
#define OSPI_PSRAM_APS6408_READ_REG_CMD           0x40U       /* Mode Register Read                  */
#define OSPI_PSRAM_APS6408_WRITE_REG_CMD          0xC0U       /* Mode Register Write                 */

#ifdef __cplusplus
}
#endif

#endif /* OSPI_PSRAM_APS6408_H_ */
