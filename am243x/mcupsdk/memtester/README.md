# AM243x Memory Tester (memtester)

## Overview

The Memory Tester is a comprehensive memory stress testing application for the AM243x processor (Cortex-R5 core) running on the AM243x EVM. It performs extensive tests on DDR memory to validate its functionality and detect potential memory defects.

## Purpose

This application tests DDR memory integrity using a suite of memory test algorithms. It's designed to:
- Validate memory hardware functionality after initialization
- Detect stuck address lines, bit flips, and memory defects
- Perform comprehensive pattern-based memory testing
- Ensure memory reliability through exhaustive testing scenarios

## Features

### Memory Test Suite

The application includes 18 comprehensive memory test algorithms:

1. **Stuck Address Test** - Detects stuck memory address lines
   - Tests if each address line properly toggles and stores values

2. **Random Value Test** - Fills memory with random patterns and verifies integrity
   - Ensures data can be stored and retrieved correctly with random values

3. **Bitwise Operations Tests** - Perform various bitwise operations and verify results
   - XOR Comparison Test
   - Subtraction Comparison Test
   - Multiplication Comparison Test
   - Division Comparison Test
   - Bitwise OR Comparison Test
   - Bitwise AND Comparison Test

4. **Sequential & Pattern Tests**
   - Sequential Increment Test - Verifies sequential data patterns
   - Solid Bits Test - Tests with patterns of all 1s and all 0s (64 iterations)
   - Checkerboard Test - Tests with alternating bit patterns (64 iterations)
   - Block Sequence Test - Tests with block-based byte patterns (256 iterations)

5. **Bit Walking Tests** - Validates individual bit transitions
   - Walking Bits 0 Test - Single bit walking through all positions
   - Walking Bits 1 Test - Inverted walking bit patterns
   - Bitspread Test - Tests pairs of bits spread apart
   - Bit Flip Test - Alternating bit patterns with multiple iterations

6. **Wide Data Tests** - Tests non-word-aligned memory access patterns
   - 8-bit Wide Random Test - Tests 8-bit access patterns
   - 16-bit Wide Random Test - Tests 16-bit access patterns

### Memory Configuration

- **DDR Base Address**: 0x80000000
- **DDR Mid Address**: 0xC0000000 (used for comparison testing)
- **Total Test Size**: 2GB (0x20000000 words, each word = 4 bytes)
- **Test Region**: Each test operates on half of total memory for comparison tests

## Building

### Prerequisites

1. MCU+ SDK v12.00.00.26 installed and configured
2. TI ARM Clang v4.0.4 toolchain available
3. SysConfig tool v1.26.0 for configuration generation

### Build Steps

Copy the example to MCU+SDK's example folder.

```bash
cd ${MCU+SDK root}

cd examples/memtester/am243x-evm/r5fss0-0_nortos/ti-arm-clang

# Release build (default)
make PROFILE=release

# Debug build
make PROFILE=debug

# Clean build artifacts
make clean

# Full scrub (removes all generated files)
make scrub
```

## Memory Map

The application uses the following memory regions:

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| R5F_VECS | 0x00000000 | 64B | Exception vectors |
| R5F_TCMA | 0x00000040 | 32KB-64B | R5F Tightly-Coupled Memory A |
| R5F_TCMB0 | 0x00080000 | 32KB | R5F Tightly-Coupled Memory B |
| NON_CACHE_MEM | 0x70060000 | 32KB | Non-cacheable memory |
| MSRAM | 0x70080000 | 256KB | Main Shared RAM |
| FLASH | 0x60100000 | 512KB | Flash memory |
| DDR (Test Region A) | 0x80000000 | 1GB | Primary test buffer |
| DDR (Test Region B) | 0xC0000000 | 1GB | Secondary comparison buffer |
