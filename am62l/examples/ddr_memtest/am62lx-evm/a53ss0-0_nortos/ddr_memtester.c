#include <stdint.h>
#include <stdio.h>
#include "ddr_memtester.h"

uint8_t ddr_read_write(uint64_t pattern) {

    volatile uint8_t status = 0x0;
    
    for(volatile uint64_t i=0; i<DDR_2_MB_SIZE; i++)
    {
        *((uint64_t *)(ADDR_DDR_MEM_START + i)) = pattern;
    }
    for(volatile uint64_t i=0; i<DDR_2_MB_SIZE; i++) 
    {
        if(*((uint64_t *)(ADDR_DDR_MEM_START + i)) != pattern)
        {
            status = 0x1;
            break;
        }
    }

    return status;

}

uint8_t ddr_memtester(int index) {

    uint8_t status = 0x0;

    static uint64_t patterns[] = PATTERNS;

    volatile uint8_t length = sizeof(patterns) / sizeof(patterns[0]);

    if(index == -1) {
        for(int k = 0; k < length; k++)
        {
            status = ddr_read_write(patterns[k]);
        }
    }
    else {
        status = ddr_read_write(patterns[index]);
    }

    return status;
}