/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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


/**
 * @file DDR_API.c
 * @brief DDR Memory API implementation for TI AM2432 Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements DDR communication APIs including testing, read, write operations.
 */

#include <stdint.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

#define ADDR_DDR_MEM_START  0x80000000UL
#define ADDR_DDR_MEM_MID    0xC0000000UL
#define ADDR_DDR_MEM_END    0xFFFFFFFFUL
/* the given test size below is used to loop over 2MB wrt writing 8 Bytes at a time */
#define DDR_TEST_SIZE       0x40000
#define DDR_END_TEST_ADDR   (ADDR_DDR_MEM_END - (DDR_TEST_SIZE << 3))
static const uint64_t gDdrMemtestPatterns[] = 
{
    0x00ULL, 
    0xffffffffffffffffULL,
    0x5555555555555555ULL,
    0xaaaaaaaaaaaaaaaaULL,
    0x1111111111111111ULL,
    0x2222222222222222ULL,
    0x4444444444444444ULL,
    0x8888888888888888ULL,
    0x3333333333333333ULL,
    0x6666666666666666ULL,
    0x9999999999999999ULL,
    0xccccccccccccccccULL,
    0x7777777777777777ULL,
    0xbbbbbbbbbbbbbbbbULL,
    0xddddddddddddddddULL,
    0xeeeeeeeeeeeeeeeeULL,
    0x7a6c7258554e494cULL,
};

static int32_t ddr_pattern_read_write(uint64_t pattern, uint64_t *start_addr, uint32_t blockSize) 
{

    int32_t status = SystemP_SUCCESS;
    
    for(uint64_t i=0; i<blockSize; i++)
    {
        *(start_addr + i) = pattern;
    }
    for(uint64_t i=0; i<blockSize; i++) 
    {
        if(*(start_addr + i) != pattern)
        {
            status = SystemP_FAILURE;
            DebugP_logError("Failed at address: 0x%llx\r\n", (uint64_t)(start_addr + i));
            break;
        }
    }

    return status;

}

static int32_t ddr_pattern_test(uint64_t *start_addr, uint32_t blockSize) 
{

    int32_t status = SystemP_SUCCESS;
    const uint32_t length = sizeof(gDdrMemtestPatterns) / sizeof(gDdrMemtestPatterns[0]);
    
    DebugP_logInfo("Testing all patterns\r\n");
    for(uint32_t i = 0; i < length; i++)
    {
        DebugP_logInfo("Testing for pattern: 0x%llx\r\n", gDdrMemtestPatterns[i]);
        status = ddr_pattern_read_write(gDdrMemtestPatterns[i], start_addr, blockSize);
        if(status == SystemP_FAILURE)
        {
            DebugP_logError("Pattern: 0x%llx failed\r\n", gDdrMemtestPatterns[i]);
            break;
        }
        DebugP_logInfo("Pattern: 0x%llx passed\r\n", gDdrMemtestPatterns[i]);
    }

    return status;
}

int32_t DDR_Memtester(uint32_t *offset, uint32_t blockSize) {

    int32_t status = SystemP_SUCCESS;
    uint64_t *startAddress = (uint64_t*)((uint64_t)offset + (uint64_t)ADDR_DDR_MEM_START);
    uint64_t *endAddress = (uint64_t*)(ADDR_DDR_MEM_END);
    if((uint64_t *)startAddress >= (uint64_t *)endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Invalid Offset for DDR starting address.\r\n");
        return status;
    }
    if((uint64_t *)(blockSize + startAddress) > (uint64_t*)endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Block size exceeds the available memory range.\r\n");
        return status;
    }

    DebugP_logInfo("Starting DDR Memory Testing\r\n");

    DebugP_logInfo("Testing from start address: 0x%p with block size(x8) : 0x%X bytes\r\n", startAddress, blockSize*8);
    status = ddr_pattern_test(startAddress, blockSize);
    if(status == SystemP_FAILURE)
    {
        DebugP_logError("DDR Memtester failed at start DDR address\r\n");
        return status;
    }

    DebugP_logInfo("All tests have passed!!\r\n");
    return status;

}

int32_t DDR_ReadWrite(uint32_t *offset, uint8_t nBytes, uint8_t *writeData, uint8_t *readData)
{
    int32_t status = SystemP_SUCCESS;
    /* 
       Size of uint64_t* on R5F processor is only 4 bytes, 
       hence using uint64_t data type instead of pointer. 
    */
    uint64_t startAddress = (uint64_t)offset + (uint64_t)ADDR_DDR_MEM_START;
    uint64_t endAddress = (uint64_t)(ADDR_DDR_MEM_END);
    if(startAddress >= endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Offset is out of available DDR memory range.\r\n");
        return status;
    }
    if((nBytes + startAddress) > endAddress)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Data size exceeds the available DDR memory range.\r\n");
        return status;
    }
    if(nBytes <= 0 || nBytes > 128)
    {
        status = SystemP_FAILURE;
        DebugP_logError("Data size should be in range 1 to 128 bytes.\r\n");
        return status;
    }

    /*Writing Data to DDR */
    for(uint8_t i=0; i<nBytes; i++)
    {
        *(uint64_t*)(startAddress + i) = writeData[i];
    }
    DebugP_logInfo("Data Successfully Written at Memory Address : 0x%p\r\n", startAddress);

    /*Reading Data from DDR */
    for(uint8_t i=0; i<nBytes; i++) 
    {
        readData[i] = *(uint64_t *)(startAddress + i);
    }
    DebugP_logInfo("Data Successfully Read from Memory Address : 0x%p\r\n", startAddress);

    return status;
}
