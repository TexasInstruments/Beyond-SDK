# TI Board Diagnostic Package

## Overview

The Board Diagnostic package for AM243x is used to run the diagnosis, test & validate functionality of supported IPs.
It is a bare-metal embedded software framework designed to validate and exercise the multiple on-chip peripherals of the AM243x microcontroller.

## Purpose

This application run the diagnosis for IPs on AM243x. It's designed to:
- Validate peripheral functionality on AM243x boards.
- Evaluate device communication interfaces.
- Generate control signals and measure responses.
- Collect diagnostic data through UART communication.
- Perform board-level validation before production.

## Building

### Prerequisites

1. MCU+ SDK v12.00.00.26 installed and configured
2. TI ARM Clang v4.0.4 toolchain available
3. SysConfig tool v1.26.0 for configuration generation

### Build Steps

Copy the example to MCU+SDK's example folder.

```bash
cd ${MCU+SDK root}

cd examples/ti_board_diagnostic/am243x-evm/r5fss0-0_nortos/ti-arm-clang

# Release build (default)
make PROFILE=release

# Debug build
make PROFILE=debug

# Clean build artifacts
make clean

# Full scrub (removes all generated files)
make scrub
```
