# OSPI PSRAM Support for AM64x MCU+ SDK

The following examples add support for OSPI PSRAM (APS6408L) on the AM64x EVM platform.

## Overview

This implementation enables the use of external PSRAM connected via OSPI interface on the AM64x platform. The PSRAM can be used for expanding available memory for applications, XIP (Execute-In-Place), or data storage. This is particularly useful when the on-chip RAM available is not sufficient to run the entire application and using DDR is too expensive.

## Hardware Support

- **EVM**: AM64x-EVM
- **Device**: AM6442
- **PSRAM Device**: APS6408L-OBMx (AP Memory)
- **Memory Size**: 8 MBytes (64 Mbits)
- **Interface**: OSPI (Octal SPI)
- **Protocol**: 8D-8D-8D (Octal DDR)

## Hardware Setup

**NOTE:** The PSRAM is mounted only on the AM64x-EVM boards following the PROC101E1_OSPI Schematic. This custom board has PSRAM and Flash attached to the I/O expander detailed below, which is connected to the OSPI controller present inside the SoC.
The datasheet for the OSPI PSRAM and the schematic for the custom baord can be found in the Docs folder.

### I/O Expander Configuration

The AM64x EVM uses a TCA6424 I/O expander (pin 0x13) to multiplex the OSPI interface between Flash and PSRAM:

- **Pin HIGH**: OSPI connected to PSRAM
- **Pin LOW**: OSPI connected to Flash (default)

## Directory Structure

```
mcupsdk/ospi_psram/
├── patches/
│   ├── 0001-am64x-Improve-OSPI-DAC-Writes-functionality.patch
│   └── 0002-am64x-Add-OSPI-PSRAM-driver.patch
│
├── Docs/
│   ├── APM_PSRAM_OPI_Xccela_APS6408L_OBMx_v3_5b_PKG-1954880.pdf
│   └── PROC101E1_OSPI_SCH_AM64x.pdf
│
├── examples/
│   ├── drivers/
│   │   ├── boot/
│   │   │   ├── sbl_ospi_psram/
│   │   │   │   └── am64x-evm/
│   │   │   │       └── r5fss0-0_nortos/
│   │   │   │           ├── main.c
│   │   │   │           ├── ospi_psram_config.c
│   │   │   │           ├── ospi_psram_config.h
│   │   │   │           ├── ti_board_psram_open_close.c
│   │   │   │           ├── ti_board_psram_open_close.h
│   │   │   │           ├── example.syscfg
│   │   │   │           └── makefile
│   │   │   └── sbl_sd_psram/
│   │   │       └── am64x-evm/
│   │   │           └── r5fss0-0_nortos/
│   │   └── ospi/
│   │       └── ospi_psram_io/
│   │           └── am64x-evm/
│   │               └── r5fss0-0_nortos/
│   │                   ├── ospi_psram_io.c
│   │                   ├── ti_board_psram_open_close.c
│   │                   ├── ti_board_psram_open_close.h
│   │                   └── makefile
│   └── hello_world_psram/
│       ├── hello_world.c
│       └── am64x-evm/
│           └── r5fss0-0_nortos/
│
├── source/
│   ├── board/
│   │   ├── psram.h
│   │   ├── ram/
│   │   │   ├── psram.c
│   │   │   └── ospi/
│   │   │       ├── io_expander.c
│   │   │       ├── ospi_psram_aps6408.h
│   │   │       ├── psram_ospi.c
│   │   │       └── psram_ospi.h
│   │   └── makefile.am64x.r5f.ti-arm-clang.nortos
│   └── drivers/
│       └── ospi/v0/lld/
│           ├── ospi_lld.h
│           └── ospi_v0_lld.c
└── tools/
    └── boot/
        └── sbl_prebuilt/
            └── am64x-evm/
                └── default_sbl_ospi_psram.cfg
```

## Patches Included

### Patch 1: Improve OSPI DAC Writes Functionality

**File**: `0001-am64x-Improve-OSPI-DAC-Writes-functionality.patch`

Optimizes OSPI Direct Access Controller (DAC) write operations for PSRAM by replacing byte-by-byte writes with memcpy() and fixing write dummy cycle configuration.

### Patch 2: Add OSPI PSRAM Driver

**File**: `0002-am64x-Add-OSPI-PSRAM-driver.patch`

Adds complete OSPI PSRAM board-level driver support including generic PSRAM API, OSPI-specific driver, APS6408 device definitions, and I/O expander support for multiplexing between Flash and PSRAM.

## Applying the Patches

To apply the patches, perform the following steps:
1. Install a fresh mcu_plus_sdk_am64x_12_00_00_27 SDK.
2. Apply the 2 patches present in the patches folder.
3. Rebuild all the libraries in release and debug modes using make/gmake commands.


## Examples Overview

### 1. hello_world_psram

**Location**: `examples/hello_world_psram/`

**Description**: This example is used to demonstrate how code execution is carried out with PSRAM. The .rodata and .text sections are placed in the PSRAM by the SBL OSPI example.

**Target**: R5FSS0-0 core (bare-metal/nortos)

**Purpose**: Showcases the configuration of the memory regions required to enable code execution from PSRAM.

### 2. ospi_psram_io

**Location**: `examples/drivers/ospi/ospi_psram_io/`

**Description**: Demonstrates OSPI PSRAM read/write operations and performance benchmarking. Tests 512KB data transfer with 10 iterations, measuring read/write speeds and verifying data integrity.

**Target**: R5FSS0-0 (bare-metal/nortos)

**Purpose**: Tests the communication between the OSPI controller present in the SoC and the PSRAM.


### 3. sbl_ospi_psram

**Location**: `examples/drivers/boot/sbl_ospi_psram/`

**Description**: Secondary Bootloader (SBL) that boots applications from OSPI Flash with support for loading XIP (Execute-In-Place) applications into PSRAM.

**Target**: R5FSS0-0 (bare-metal/nortos)

**Purpose**: Showcases how the code execution is enabled and carried out by the bootloader by dynamically switching between the Flash and PSRAM and transferring the required sections from the Flash to the PSRAM.

### 4. sbl_sd_psram

**Location**: `examples/drivers/boot/sbl_sd_psram/`

**Description**: Secondary Bootloader (SBL) that boots applications from SD card with support for loading XIP (Execute-In-Place) applications into PSRAM.

**Target**: R5FSS0-0 (bare-metal/nortos)

**Purpose**: Showcases how the code execution is enabled and carried out by the bootloader by loading the sections from the SD card and transferring them to PSRAM.

## Building and Running the Examples

- After applying the 2 patches and rebuilding the libraries, the examples detailed above can be built.

### Steps to run the hello_world example

1. SBL OSPI PSRAM:
- After building the example, use the format specified in the default_sbl_ospi_psram.cfg present under tools/boot/sbl_prebuilt/am64x-evm/ to flash the .mcelf.hs_fs and the .mcelf_xip images at the mentioned offsets.
- After flashing the binaries in UART bootmode, power off the EVM, switch to the OSPI bootmode and power on the EVM.

2. SBL SD PSRAM:
- In the SD card, copy the tiboot3.bin file from the sbl_sd example. Rename the .mcelf.hs_fs binary to "app" and .mcelf_xip binary to "app_xip" and copy them to the SD card.
- After copying the required binaries, insert the SD card in the SD card slot of the EVM and power on the EVM in SD bootmode.

### Steps to run the ospi_psram_io example

1. SBL NULL:
- Flash the default SBL NULL binary.
- After building the ospi_psram_io example, connect the target to CCS and load and run the .out file of the example.

2. SBL OSPI PSRAM:
- After building the example, use the format specified in the default_sbl_ospi_psram.cfg present under tools/boot/sbl_prebuilt/am64x-evm/ to flash the .mcelf.hs_fs and the .mcelf_xip images at the mentioned offsets.
- After flashing the binaries in UART bootmode, power off the EVM, switch to the OSPI bootmode and power on the EVM.

3. SBL SD PSRAM:
- In the SD card, copy the tiboot3.bin file from the sbl_sd example. Rename the .mcelf.hs_fs binary to "app" and .mcelf_xip binary to "app_xip" and copy them to the SD card.
- After copying the required binaries, insert the SD card in the SD card slot of the EVM and power on the EVM in SD bootmode.

## Notes

1. All the examples and patches are compatible with the mcu_plus_sdk_am64x_12_00_00_27 MCU+ SDK.
2. The OSPI interface is shared between Flash and PSRAM (time-multiplexed via I/O expander).
3. Only one device (Flash or PSRAM) can be accessed at a time.
4. Write operations must be even-aligned with minimum 2-byte length.
5. The above examples only demonstrate the enablement and working of code execution from PSRAM. The user or the programmer is free to change the size of the buffers and the value of the offsets.

## References

- AM64x MCU+ SDK [https://www.ti.com/tool/download/MCU-PLUS-SDK-AM64X/12.00.00.27]
- AM64x Technical Reference Manual [https://www.ti.com/lit/pdf/SPRUIM2]
- The datasheet for the OSPI PSRAM and the schematic for the custom baord can be found in the Docs folder.

