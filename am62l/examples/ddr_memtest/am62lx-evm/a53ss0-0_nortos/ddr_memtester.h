#include <stdint.h>
#define ADDR_DDR_MEM_START ((volatile uint64_t *)0x83000000)
#define DDR_2_MB_SIZE 0x40000
#define PATTERNS {          \
        0x00,               \
        0xffffffffffffffff, \
        0x5555555555555555, \
        0xaaaaaaaaaaaaaaaa, \
        0x1111111111111111, \
        0x2222222222222222, \
        0x4444444444444444, \
        0x8888888888888888, \
        0x3333333333333333, \
        0x6666666666666666, \
        0x9999999999999999, \
        0xcccccccccccccccc, \
        0x7777777777777777, \
        0xbbbbbbbbbbbbbbbb, \
        0xdddddddddddddddd, \
        0xeeeeeeeeeeeeeeee, \
        0x7a6c7258554e494c, \
    }

uint8_t ddr_memtester(int index);