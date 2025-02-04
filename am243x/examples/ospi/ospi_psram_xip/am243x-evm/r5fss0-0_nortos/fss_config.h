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

#ifndef FSS_CONFIG_H_
#define FSS_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/ospi.h>

#define PAGE_SIZE_8MB           0x800000
#define BLOCK_SIZE_8MB          0x800000

#define OCTAL_SPI_MODE          3

#define DTR_PROTOCOL_EN         1
#define DTR_PROTOCOL_DIS        0

///STIG REG Defines///////////////
#define STIG_MEM_BANK_EN         1
#define STIG_MEM_BANK_DIS        0

#define DUMMY_CYC0              0
#define DUMMY_CYC1              1
#define DUMMY_CYC2              2
#define DUMMY_CYC3              3
#define DUMMY_CYC4              4
#define DUMMY_CYC5              5
#define DUMMY_CYC6              6
#define DUMMY_CYC7              7
#define DUMMY_CYC8              8
#define DUMMY_CYC9              9
#define DUMMY_CYC10             10

#define DATA_BYTE_CNT1          0
#define DATA_BYTE_CNT2          1
#define DATA_BYTE_CNT3          2
#define DATA_BYTE_CNT4          3
#define DATA_BYTE_CNT5          4
#define DATA_BYTE_CNT6          5
#define DATA_BYTE_CNT7          6
#define DATA_BYTE_CNT8          7

#define ADDR_BYTE_CNT1          0
#define ADDR_BYTE_CNT2          1
#define ADDR_BYTE_CNT3          2
#define ADDR_BYTE_CNT4          3

#define RD_CMD                  1
#define WR_CMD                  0
#define NO_DATA_CMD             2

#define MODE_BIT_EN             1
#define MODE_BIT_DIS            0

#define CMD_ADDR_EN             1
#define CMD_ADDR_DIS            0

#define POLL_DIS                1
#define POLL_EN                 0

///////////////////////////////////

#define PHY_EN                  1
#define PHY_DIS                 0

#define AUTO_WEL_EN             0
#define AUTO_WEL_DIS            1

#define DBOPCODE                0

#define POLL_CNT0               0
#define POLL_DLY0               0
#define POLL_POL_LOW            0
#define POLL_POL_HIGH           1
#define POLL_BIT0               0

#define CMD_RDSR_OP1            0x05
#define CMD_RDSR_OP2            0xFA

#define DB_OPCODE_MODE_EN       1
#define DB_OPCODE_MODE_DIS      0

#define DAC_EN                  1
#define DAC_DIS                 0

void ospi_read_instr_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t instr_type,uint8_t ddr_bit_en,uint8_t mode_bit_en,uint8_t dummy_cyc_req,uint8_t phy_en );
void ospi_wr_instr_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t wel_dis,uint8_t dummy_cyc_req );
void ospi_cs_dly_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t cs_deasrt_dly,uint8_t cs_deasrt_diff_slv,uint8_t cs_eoc_dly,uint8_t cs_soc_dly );
void ospi_poll_status_from_flash_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t poll_delay,uint8_t poll_cnt,uint8_t poll_dis,uint8_t poll_polarity,uint8_t pol_bit_index,uint8_t dbopcode_en,uint8_t dummy_cyc  );

void ospi_stig_cmd_ctrl(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t mem_bank_en,uint8_t mode_bit_en,uint8_t dummy_cyc,uint8_t addr_bytes,uint8_t cmd_addr_en );
void ospi_stig_cmd_ctrl_data_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t rd_wrn,uint8_t data_bytes );
void ospi_stig_cmd_addr(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t addr);
void ospi_stig_cmd_opcode(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t dbopcode_en,uint8_t ospi_opcode1,uint8_t ospi_opcode2);          
void ospi_stig_cmd_wait_for_done(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs);
uint32_t ospi_stig_cmd_rd_data_lower(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs);
uint32_t ospi_stig_cmd_rd_data_upper(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs);
void ospi_stig_cmd_wr_data_lower(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t data );
void ospi_stig_cmd_wr_data_upper(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t data );
void ospi_instr_type(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t instr_type );
void ospi_inst_opcode(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t dbopcode_en,uint8_t ospi_rd_opcode1,uint8_t ospi_rd_opcode2,uint8_t ospi_wr_opcode1,uint8_t ospi_wr_opcode2);
void ospi_dac_en(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t ospi_dac_en);
int32_t Psram_ospiread_write_cmd(Psram_Config *config, uint32_t offset, uint8_t *pSrc, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif /* FSS_CONFIG_H__ */
