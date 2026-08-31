/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0L130X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0L130X
#define CONFIG_MSPM0L1306

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM10)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM9)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM10_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM9_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_ADC12_0_C8_PORT                                               GPIOA
#define GPIO_ADC12_0_C8_PIN                                       DL_GPIO_PIN_16
#define GPIO_ADC12_0_IOMUX_C8                                    (IOMUX_PINCM17)
#define GPIO_ADC12_0_IOMUX_C8_FUNC                (IOMUX_PINCM17_PF_UNCONNECTED)



/* Port definition for Pin Group GPIO_GRP_0 */
#define GPIO_GRP_0_PORT                                                  (GPIOA)

/* Defines for XN: GPIOA.15 with pinCMx 16 on package pin 19 */
#define GPIO_GRP_0_XN_PIN                                       (DL_GPIO_PIN_15)
#define GPIO_GRP_0_XN_IOMUX                                      (IOMUX_PINCM16)
/* Defines for XP: GPIOA.17 with pinCMx 18 on package pin 21 */
#define GPIO_GRP_0_XP_PIN                                       (DL_GPIO_PIN_17)
#define GPIO_GRP_0_XP_IOMUX                                      (IOMUX_PINCM18)
/* Defines for YN: GPIOA.16 with pinCMx 17 on package pin 20 */
#define GPIO_GRP_0_YN_PIN                                       (DL_GPIO_PIN_16)
#define GPIO_GRP_0_YN_IOMUX                                      (IOMUX_PINCM17)
/* Defines for YP: GPIOA.25 with pinCMx 26 on package pin 29 */
#define GPIO_GRP_0_YP_PIN                                       (DL_GPIO_PIN_25)
#define GPIO_GRP_0_YP_IOMUX                                      (IOMUX_PINCM26)
/* Defines for YP2: GPIOA.21 with pinCMx 22 on package pin 25 */
#define GPIO_GRP_0_YP2_PIN                                      (DL_GPIO_PIN_21)
#define GPIO_GRP_0_YP2_IOMUX                                     (IOMUX_PINCM22)
/* Defines for YN2: GPIOA.24 with pinCMx 25 on package pin 28 */
#define GPIO_GRP_0_YN2_PIN                                      (DL_GPIO_PIN_24)
#define GPIO_GRP_0_YN2_IOMUX                                     (IOMUX_PINCM25)
/* Defines for YN3: GPIOA.22 with pinCMx 23 on package pin 26 */
#define GPIO_GRP_0_YN3_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_GRP_0_YN3_IOMUX                                     (IOMUX_PINCM23)
/* Defines for YP3: GPIOA.1 with pinCMx 2 on package pin 2 */
#define GPIO_GRP_0_YP3_PIN                                       (DL_GPIO_PIN_1)
#define GPIO_GRP_0_YP3_IOMUX                                      (IOMUX_PINCM2)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_ADC12_0_init(void);



#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
