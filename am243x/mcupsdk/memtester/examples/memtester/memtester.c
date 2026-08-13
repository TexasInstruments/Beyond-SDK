#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "memtester.h"

#define ONE 0x00000001UL

#define FMT_TARGET "0x%08lx"
#define UL_ONEBITS 0xffffffff
#define UL_LEN 32
#define CHECKERBOARD1 0x55555555
#define CHECKERBOARD2 0xaaaaaaaa
#define UL_BYTE(x) ((x | x << 8 | x << 16 | x << 24))

#define DDR_BASE_ADDRESS 0x80000000
#define DDR_MID_ADDRESS  0xC0000000
#define TOTAL_WORDS      0x20000000 // Each word is 4 bytes, considering the DDR size to be 2GB

union {
    uint8_t bytes[UL_LEN/8];
    uint32_t val;
} mword8;

union {
    unsigned short u16s[UL_LEN/16];
    uint32_t val;
} mword16;

/* Function definitions. */

uint32_t rand_ul()
{
    static volatile uint32_t random1 = 0xFFFF0FFF, random2 = 0x50;       // Numbers used for generation of random values
    static volatile uint32_t seed = 0x678;                               // Initial seed value

    /* Overflow condition is not checked as we just want random number so wrapped value is always accepted */
    random1 = (random1 * seed + random2);   
    random2 = (random2 * 7) + 0x56AB34;
    seed = (seed * random1) + 0x41EA45;                                                    
    return random1;
}

int compare_regions(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    int status = SystemP_SUCCESS;
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;

    for (uint32_t i = 0; i < count; i++, p1++, p2++) {
        if (*p1 != *p2) {
            DebugP_logError("Failure: " FMT_TARGET " != " FMT_TARGET " at physical address " FMT_TARGET ".\r\n", p1, p2, (DDR_BASE_ADDRESS + (i * sizeof(uint32_t))));
            status = SystemP_FAILURE;
        }
    }
    return status;
}

int test_stuck_address(volatile uint32_t *bufa, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    DebugP_logInfo("Write the bitwise inverted memory address at odd values of i+j, otherwise exact memory address\r\n");
    for (uint32_t j = 0; j < 16; j++) {
        p1 = (volatile uint32_t *) bufa;
        DebugP_logInfo("Setting value %u time\r\n", j+1);
        for (uint32_t i = 0; i < count; i++) {
            *p1 = ((j + i) % 2) == 0 ? (uint32_t) p1 : ~((uint32_t) p1);
            p1++;
        }
        
        DebugP_logInfo("Testing %u time\r\n", j+1);
        p1 = (volatile uint32_t *) bufa;
        for (uint32_t i = 0; i < count; i++, p1++) {
            if (*p1 != (((j + i) % 2) == 0 ? (uint32_t) p1 : ~((uint32_t) p1))) {
                DebugP_logError("Failure: possible bad address line at physical address " FMT_TARGET ".\n", p1);
                return SystemP_FAILURE;
            }
        }
    }
    return SystemP_SUCCESS;
}

int test_random_value(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t i;

    DebugP_logInfo("Filling random value to DDR.\r\n");
    for (i = 0; i < count; i++) {
        *p1++ = *p2++ = rand_ul();
    }
    return compare_regions(bufa, bufb, count);
}

int test_xor_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Bitwise XOR Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ ^= q;
        *p2++ ^= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_sub_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Subtract Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ -= q;
        *p2++ -= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_mul_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Multiplication Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ *= q;
        *p2++ *= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_div_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Division Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        if (!q) {
            q++;
        }
        *p1++ /= q;
        *p2++ /= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_or_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Bitwise OR Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ |= q;
        *p2++ |= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_and_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Bitwise AND Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ &= q;
        *p2++ &= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_seqinc_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    uint32_t q = rand_ul();

    DebugP_logInfo("Sequential increment Operation Test.\r\n");
    for (uint32_t i = 0; i < count; i++) {
        *p1++ = *p2++ = (i + q);
    }
    return compare_regions(bufa, bufb, count);
}

int test_solidbits_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t q;
    uint32_t i;
    DebugP_logInfo("Solid bits Operation Test.\r\n");
    for (j = 0; j < 64; j++) {
        q = (j % 2) == 0 ? UL_ONEBITS : 0;
        DebugP_logInfo("Setting Solid bits %u time\r\n", j+1);
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }

    return SystemP_SUCCESS;
}

int test_checkerboard_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t q;
    uint32_t i;

    DebugP_logInfo("Checkerboard Operation Test.\r\n");
    for (j = 0; j < 64; j++) {
        q = (j % 2) == 0 ? CHECKERBOARD1 : CHECKERBOARD2;
        DebugP_logInfo("Setting checkerboard %u time\r\n", j+1);
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_blockseq_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t i;

    DebugP_logInfo("Block Sequence Operation Test.\r\n");
    for (j = 0; j < 256; j++) {
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        DebugP_logInfo("Setting block sequence %u time\r\n", j+1);
        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (uint32_t) UL_BYTE(j);
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_walkbits0_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t i;

    DebugP_logInfo("Walkbits Operation Test.\r\n");
    for (j = 0; j < UL_LEN * 2; j++) {
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        DebugP_logInfo("Setting walkbits %u time\r\n", j+1);
        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = ONE << j;
            } else { /* Walk it back down. */
                *p1++ = *p2++ = ONE << (UL_LEN * 2 - j - 1);
            }
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        fflush(stdout);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_walkbits1_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t i;

    DebugP_logInfo("Walkbits1 Operation Test.\r\n");
    for (j = 0; j < UL_LEN * 2; j++) {
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        DebugP_logInfo("Setting walkbits1 %u time\r\n", j+1);
        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = UL_ONEBITS ^ (ONE << j);
            } else { /* Walk it back down. */
                *p1++ = *p2++ = UL_ONEBITS ^ (ONE << (UL_LEN * 2 - j - 1));
            }
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_bitspread_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j;
    uint32_t i;

    DebugP_logInfo("Bitspread Operation Test.\r\n");
    for (j = 0; j < UL_LEN * 2; j++) {
        p1 = (volatile uint32_t *) bufa;
        p2 = (volatile uint32_t *) bufb;
        DebugP_logInfo("Setting bitspread %u time\r\n", j+1);
        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = (i % 2 == 0)
                    ? (ONE << j) | (ONE << (j + 2))
                    : UL_ONEBITS ^ ((ONE << j)
                                    | (ONE << (j + 2)));
            } else { /* Walk it back down. */
                *p1++ = *p2++ = (i % 2 == 0)
                    ? (ONE << (UL_LEN * 2 - 1 - j)) | (ONE << (UL_LEN * 2 + 1 - j))
                    : UL_ONEBITS ^ (ONE << (UL_LEN * 2 - 1 - j)
                                    | (ONE << (UL_LEN * 2 + 1 - j)));
            }
        }
        DebugP_logInfo("Testing %u time\r\n", j+1);
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_bitflip_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count) {
    volatile uint32_t *p1 = bufa;
    volatile uint32_t *p2 = bufb;
    unsigned int j, k;
    uint32_t q;
    uint32_t i;

    DebugP_logInfo("Bitflip Operation Test.\r\n");
    for (k = 0; k < UL_LEN; k++) {
        q = ONE << k;
        for (j = 0; j < 8; j++) {
            q = ~q;
            DebugP_logInfo("Setting Bitflip %u time\r\n", j+1);
            p1 = (volatile uint32_t *) bufa;
            p2 = (volatile uint32_t *) bufb;
            for (i = 0; i < count; i++) {
                *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
            }
            DebugP_logInfo("Testing %u time\r\n", j+1);
            if (compare_regions(bufa, bufb, count)) {
                return SystemP_FAILURE;
            }
        }
    }
    return SystemP_SUCCESS;
}

int test_8bit_wide_random(volatile uint32_t* bufa, volatile uint32_t* bufb, uint32_t count) {
    volatile uint8_t *p1, *t;
    volatile uint32_t *p2;
    int attempt;
    unsigned int b;
    uint32_t i;

    for (attempt = 0; attempt < 2;  attempt++) {
        if (attempt & 1) {
            p1 = (volatile uint8_t *) bufa;
            p2 = bufb;
        } else {
            p1 = (volatile uint8_t *) bufb;
            p2 = bufa;
        }
        for (i = 0; i < count; i++) {
            t = mword8.bytes;
            *p2++ = mword8.val = rand_ul();
            for (b=0; b < UL_LEN/8; b++) {
                *p1++ = *t++;
            }
        }
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

int test_16bit_wide_random(volatile uint32_t* bufa, volatile uint32_t* bufb, uint32_t count) {
    volatile uint16_t *p1, *t;
    volatile uint32_t *p2;
    int attempt;
    unsigned int b;
    uint32_t i;

    for (attempt = 0; attempt < 2; attempt++) {
        if (attempt & 1) {
            p1 = (volatile uint16_t *) bufa;
            p2 = bufb;
        } else {
            p1 = (volatile uint16_t *) bufb;
            p2 = bufa;
        }
        for (i = 0; i < count; i++) {
            t = mword16.u16s;
            *p2++ = mword16.val = rand_ul();
            for (b = 0; b < UL_LEN/16; b++) {
                *p1++ = *t++;
            }
        }
        if (compare_regions(bufa, bufb, count)) {
            return SystemP_FAILURE;
        }
    }
    return SystemP_SUCCESS;
}

void stress_test()
{
    uint32_t status = SystemP_SUCCESS;
    uint32_t *bufferA = (uint32_t*)(DDR_BASE_ADDRESS), *bufferB = (uint32_t*)(DDR_MID_ADDRESS);
    uint32_t count = ((uint32_t)TOTAL_WORDS / 2);
    
    DebugP_log("Testing Stuck Memory Address\r\n");
    status = test_stuck_address(bufferA, (uint32_t)TOTAL_WORDS);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Stuck Memory Address test passed\r\n");

    DebugP_log("Testing Random Value\r\n");
    status = test_random_value(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Random Value test passed\r\n");

    DebugP_log("Testing Bitwise XOR Comparison\r\n");
    status = test_xor_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Bitwise XOR test passed\r\n");

    DebugP_log("Testing Subtraction Comparison\r\n");
    status = test_sub_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Subtraction Comparison test passed\r\n");

    DebugP_log("Testing Multiplication Comparison\r\n");
    status = test_mul_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Multiplication Comparison test passed\r\n");

    DebugP_log("Testing Divison Comparison\r\n");
    status = test_div_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Division Comparison test passed\r\n");

    DebugP_log("Testing Bitwise OR Comparison\r\n");
    status = test_or_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Bitwise OR Comparison test passed\r\n");

    DebugP_log("Testing Bitwise AND Comparison\r\n");
    status = test_and_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Bitwise AND Comparison test passed\r\n");

    DebugP_log("Testing Sequence Increment Comparison\r\n");
    status = test_seqinc_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Sequence Increment Comparison test passed\r\n");

    DebugP_log("Testing Solidbits Comparison\r\n");
    status = test_solidbits_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Solidbits Comparison passed\r\n");

    DebugP_log("Testing Checkerboard Comparison\r\n");
    status = test_checkerboard_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Checkerboard Comparison test passed\r\n");

    DebugP_log("Testing Block Sequence Comparison\r\n");
    status = test_blockseq_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Block Sequence Comparison test passed\r\n");

    DebugP_log("Testing Walkbit0 Comparison\r\n");
    status = test_walkbits0_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Walkbit0 Comparison test passed\r\n");

    DebugP_log("Testing Walkbit1 Comparison\r\n");
    status = test_walkbits1_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Walkbit1 Comparison test passed\r\n");

    DebugP_log("Testing Bitspread Comparison\r\n");
    status = test_bitspread_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Bitspread Comparison test passed\r\n");

    DebugP_log("Testing Bitflip Comparison\r\n");
    status = test_bitflip_comparison(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("Bitflip Comparison test passed\r\n");

    DebugP_log("Testing 8 bit wide random Comparison\r\n");
    status = test_8bit_wide_random(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("8 bit wide random Comparison test passed\r\n");

    DebugP_log("Testing 16 bit wide random Comparison\r\n");
    status = test_16bit_wide_random(bufferA, bufferB, count);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_log("16 bit wide random Comparison test passed\r\n");

}

void memtester_main()
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Start Memory Test...\r\n");
    stress_test();
    DebugP_log("All test have passed.\r\n");
    
    Board_driversClose();
    Drivers_close();
}