/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <board/psram.h>
#include "ti_drivers_open_close.h"
//#include "psram_ospi.h"
#include "fss_config.h"
#include "ti_board_psram_open_close.h"

/***************************************************************************************************************************/
/*********************************** STIG RELATED FUNCTIONS ****************************************************************/
//Function to configure the command controls for STIG mode
//Includes configuring Dummy cycle needed in command , whether command requires a ddress bytes ,num of address bytes
//mode bit enable,stig memory bank en
void ospi_stig_cmd_ctrl(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t mem_bank_en,uint8_t mode_bit_en,uint8_t dummy_cyc,uint8_t addr_bytes,uint8_t cmd_addr_en )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG;
    Ospi_Regs->FLASH_CMD_CTRL_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD_MASK  |
                                                       CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_COMD_ADDR_FLD_MASK    |
                                                       CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_ADDR_BYTES_FLD_MASK      |
                                                       CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_STIG_MEM_BANK_EN_FLD_MASK      |
                                                       CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_MODE_BIT_FLD_MASK     ))
                                     |((dummy_cyc   << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD_SHIFT) |
                                       (cmd_addr_en << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_COMD_ADDR_FLD_SHIFT)   |
                                       (addr_bytes  << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_ADDR_BYTES_FLD_SHIFT)   |
                                       (mem_bank_en  << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_STIG_MEM_BANK_EN_FLD_SHIFT)   |
                                       (mode_bit_en << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_MODE_BIT_FLD_SHIFT)     ));


}

void ospi_stig_cmd_ctrl_data_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t rd_wrn,uint8_t data_bytes )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG;
    if(rd_wrn == 0x1)
    {
        Ospi_Regs->FLASH_CMD_CTRL_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_RD_DATA_BYTES_FLD_MASK   |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_READ_DATA_FLD_MASK       |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_WRITE_DATA_FLD_MASK     ))
                                    | ((data_bytes << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_RD_DATA_BYTES_FLD_SHIFT)      |
                                       (0x1 << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_READ_DATA_FLD_SHIFT)       ) );
    }

    if(rd_wrn == 0x0)
    {
        Ospi_Regs->FLASH_CMD_CTRL_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_WR_DATA_BYTES_FLD_MASK   |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_READ_DATA_FLD_MASK       |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_WRITE_DATA_FLD_MASK      ))
                                    | ((data_bytes << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_WR_DATA_BYTES_FLD_SHIFT)      |
                                       (0x1 << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_WRITE_DATA_FLD_SHIFT)       ) );
    }
    if(rd_wrn == 0x2)
    {
        Ospi_Regs->FLASH_CMD_CTRL_REG = (reg_read_val & ~(CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_WR_DATA_BYTES_FLD_MASK   |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_RD_DATA_BYTES_FLD_MASK   |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_READ_DATA_FLD_MASK       |
                                                           CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_WRITE_DATA_FLD_MASK      ));

    }
}


//Function is used to configure the address to be used in the STIG command
void ospi_stig_cmd_addr(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t addr)
{
    //uint32_t reg_read_val;
    //reg_read_val = Ospi_Regs->FLASH_CMD_ADDR_REG;

    Ospi_Regs->FLASH_CMD_ADDR_REG = addr;

}

//Function is used to configure the OPCODE to be used in the STIG command
//After programming opcode, STIG command is also triggered using FLASH CMD CTRL REG
void ospi_stig_cmd_opcode(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t dbopcode_en,uint8_t ospi_opcode1,uint8_t ospi_opcode2)
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG;
    Ospi_Regs->FLASH_CMD_CTRL_REG = ( (reg_read_val & ~CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_OPCODE_FLD_MASK ) | (ospi_opcode1 << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_OPCODE_FLD_SHIFT) );

    reg_read_val = Ospi_Regs->OPCODE_EXT_LOWER_REG;
    if(dbopcode_en == 0x1)
    {
        Ospi_Regs->OPCODE_EXT_LOWER_REG = ( (reg_read_val & ~CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_STIG_OPCODE_FLD_MASK ) | (ospi_opcode2 << CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_STIG_OPCODE_FLD_SHIFT) );
    }

    reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG;
    Ospi_Regs->FLASH_CMD_CTRL_REG = ( (reg_read_val & ~CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_EXEC_FLD_MASK ) | (0x1 << CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_EXEC_FLD_SHIFT) );

}

//Function to wait for completion of STIG Command
void ospi_stig_cmd_wait_for_done(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs)
{
    uint32_t reg_read_val;

    reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG; //PC--
    while((reg_read_val & CSL_OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_EXEC_STATUS_FLD_MASK) != 0) //PC-- From TRM we have  FLASH_CMD_CTRL_REG (offset 0x90, bit 1 (CMD_EXEC_STATUS_FLD) with Reset val =0)
    {
        reg_read_val = Ospi_Regs->FLASH_CMD_CTRL_REG;
    }
}

//Function to read STIG CMD DATA LOWER 4 bytes
uint32_t ospi_stig_cmd_rd_data_lower(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs)
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->FLASH_RD_DATA_LOWER_REG;
    return reg_read_val;
}

//Function to read STIG CMD DATA UPPER 4 bytes
uint32_t ospi_stig_cmd_rd_data_upper(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs)
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->FLASH_RD_DATA_UPPER_REG;
    return reg_read_val;
}

void ospi_stig_cmd_wr_data_lower(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t data )
{
    Ospi_Regs->FLASH_WR_DATA_LOWER_REG = data;
}

//Function to read STIG CMD DATA UPPER 4 bytes
void ospi_stig_cmd_wr_data_upper(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint32_t data )
{
    Ospi_Regs->FLASH_WR_DATA_UPPER_REG = data;
}

//Function to configure the Instruction type . Whether SPI MODE, Dual-SPI , Quad-SPI or OSPI
void ospi_instr_type(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t instr_type )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->DEV_INSTR_RD_CONFIG_REG;
    Ospi_Regs->DEV_INSTR_RD_CONFIG_REG = ((reg_read_val & ~CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_MASK) | (instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_SHIFT));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ospi_read_instr_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t instr_type,uint8_t ddr_bit_en,uint8_t mode_bit_en,uint8_t dummy_cyc_req,uint8_t phy_en )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->DEV_INSTR_RD_CONFIG_REG;
    Ospi_Regs->DEV_INSTR_RD_CONFIG_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_MASK   |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD_MASK |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_MASK |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_MASK |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_MODE_BIT_ENABLE_FLD_MASK |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD_MASK))
                                    | ((instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_SHIFT)      |
                                       (ddr_bit_en << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD_SHIFT) |
                                       (instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_SHIFT) |
                                       (instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_SHIFT) |
                                       (mode_bit_en << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_MODE_BIT_ENABLE_FLD_SHIFT) |
                                       (dummy_cyc_req << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD_SHIFT)            ) );


    reg_read_val = Ospi_Regs->DEV_INSTR_WR_CONFIG_REG;
    Ospi_Regs->DEV_INSTR_WR_CONFIG_REG =     ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_MASK  |
                                                                CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_MASK))
                                            | ((instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_SHIFT) |
                                               (instr_type << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_SHIFT)) );

    reg_read_val = Ospi_Regs->CONFIG_REG;
    Ospi_Regs->CONFIG_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD_MASK  |
                                               CSL_OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD_MASK))
                                            | ((phy_en << CSL_OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD_SHIFT) |
                                               (ddr_bit_en << CSL_OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD_SHIFT)) );
}

void ospi_wr_instr_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t wel_dis,uint8_t dummy_cyc_req )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->DEV_INSTR_WR_CONFIG_REG;
    Ospi_Regs->DEV_INSTR_WR_CONFIG_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DUMMY_WR_CLK_CYCLES_FLD_MASK   |
                                                        CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WEL_DIS_FLD_MASK ))
                                    | ((dummy_cyc_req << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DUMMY_WR_CLK_CYCLES_FLD_SHIFT)  |
                                       (wel_dis << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WEL_DIS_FLD_SHIFT)            ) );
}

void ospi_cs_dly_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t cs_deasrt_dly,uint8_t cs_deasrt_diff_slv,uint8_t cs_eoc_dly,uint8_t cs_soc_dly )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->DEV_DELAY_REG;
    Ospi_Regs->DEV_DELAY_REG =        ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_INIT_FLD_MASK   |
                                                         CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_AFTER_FLD_MASK  |
                                                         CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_BTWN_FLD_MASK   |
                                                         CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_NSS_FLD_MASK))
                                    | ((cs_soc_dly << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_INIT_FLD_SHIFT)      |
                                       (cs_eoc_dly << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_AFTER_FLD_SHIFT) |
                                       (cs_deasrt_diff_slv << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_BTWN_FLD_SHIFT) |
                                       (cs_deasrt_dly << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_NSS_FLD_SHIFT)            ) );
}

void ospi_poll_status_from_flash_cfg(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t poll_delay,uint8_t poll_cnt,uint8_t poll_dis,uint8_t poll_polarity,uint8_t pol_bit_index,uint8_t dbopcode_en,uint8_t dummy_cyc  )
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->WRITE_COMPLETION_CTRL_REG;
    Ospi_Regs->WRITE_COMPLETION_CTRL_REG = ((reg_read_val & ~(CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLL_REP_DELAY_FLD_MASK |
                                                              CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLL_COUNT_FLD_MASK |
                                                              CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_DISABLE_POLLING_FLD_MASK |
                                                              CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLLING_POLARITY_FLD_MASK |
                                                              CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLLING_BIT_INDEX_FLD_MASK |
                                                              CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_OPCODE_FLD_MASK))
                                           |  ((poll_delay << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLL_REP_DELAY_FLD_SHIFT) |
                                               (poll_cnt << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLL_COUNT_FLD_SHIFT)  |
                                               (poll_dis << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_DISABLE_POLLING_FLD_SHIFT)  |
                                               (poll_polarity << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLLING_POLARITY_FLD_SHIFT)  |
                                               (pol_bit_index << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLLING_BIT_INDEX_FLD_SHIFT)  |
                                               (CMD_RDSR_OP1 << CSL_OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_OPCODE_FLD_SHIFT)  ));


    if(dbopcode_en == 0x1)
    {
        reg_read_val = Ospi_Regs->OPCODE_EXT_LOWER_REG;
        Ospi_Regs->OPCODE_EXT_LOWER_REG = ((reg_read_val & ~CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_POLL_OPCODE_FLD_MASK) | (CMD_RDSR_OP2 << CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_POLL_OPCODE_FLD_SHIFT));
    }


    reg_read_val = Ospi_Regs->POLLING_FLASH_STATUS_REG;
    Ospi_Regs->POLLING_FLASH_STATUS_REG = ((reg_read_val & ~CSL_OSPI_FLASH_CFG_POLLING_FLASH_STATUS_REG_DEVICE_STATUS_NB_DUMMY_MASK) | (dummy_cyc << CSL_OSPI_FLASH_CFG_POLLING_FLASH_STATUS_REG_DEVICE_STATUS_NB_DUMMY_SHIFT));

}

void ospi_inst_opcode(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t dbopcode_en,uint8_t ospi_rd_opcode1,uint8_t ospi_rd_opcode2,uint8_t ospi_wr_opcode1,uint8_t ospi_wr_opcode2)
{
    uint32_t reg_read_val;

        /* Configure the Device Read Instruction Configuration Register */
        reg_read_val = Ospi_Regs->DEV_INSTR_RD_CONFIG_REG;
        Ospi_Regs->DEV_INSTR_RD_CONFIG_REG = ( (reg_read_val & ~CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD_MASK ) | (ospi_rd_opcode1 << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD_SHIFT) );
        /* Configure the Device Write Instruction Configuration Register */
        reg_read_val = Ospi_Regs->DEV_INSTR_WR_CONFIG_REG;
        Ospi_Regs->DEV_INSTR_WR_CONFIG_REG = ( (reg_read_val & ~CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD_MASK ) | (ospi_wr_opcode1 << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD_SHIFT) );

        if(dbopcode_en == 0x1)
        {
            reg_read_val = Ospi_Regs->OPCODE_EXT_LOWER_REG;
            Ospi_Regs->OPCODE_EXT_LOWER_REG = ( (reg_read_val & ~(CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_WRITE_OPCODE_FLD_MASK |
                                                                  CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_READ_OPCODE_FLD_MASK )) |
                                                (ospi_wr_opcode2 << CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_WRITE_OPCODE_FLD_SHIFT) |
                                                (ospi_rd_opcode2 << CSL_OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_READ_OPCODE_FLD_SHIFT));
        }
}

//Funct enables the DAC(Direct Access Controller) mode of OSPI
void ospi_dac_en(volatile CSL_ospi_flash_cfgRegs* Ospi_Regs,uint8_t ospi_dac_en)
{
    uint32_t reg_read_val;
    reg_read_val = Ospi_Regs->CONFIG_REG;
    Ospi_Regs->CONFIG_REG = ((reg_read_val & ~CSL_OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD_MASK) | (ospi_dac_en << CSL_OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD_SHIFT));
}
