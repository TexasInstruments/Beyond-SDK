/* Function declaration. */

int test_stuck_address(volatile uint32_t *bufa, uint32_t count);
int test_random_value(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_xor_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_sub_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_mul_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_div_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_or_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_and_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_seqinc_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_solidbits_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_checkerboard_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_blockseq_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_walkbits0_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_walkbits1_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_bitspread_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_bitflip_comparison(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_8bit_wide_random(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);
int test_16bit_wide_random(volatile uint32_t *bufa, volatile uint32_t *bufb, uint32_t count);

