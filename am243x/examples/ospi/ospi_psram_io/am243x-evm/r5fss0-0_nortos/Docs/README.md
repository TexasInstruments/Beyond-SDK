## AM243x OSPI PSRAM

### HARDWARE:

#### EVM Modification Schematic:
- PROC101E1_OSPI_SCH_AM243x.pdf

#### High Speed Switch:
- IO Expander is used to set OSPI_SW_SEL
#### Note: 
- There is a 10K pull-down on OSPI_SW_SEL to default to OSPI FLASH

#### PSRAM datasheet:
- [APM_PSRAM_OPI_Xccela_APS6408L_3OBMx_v3_5b_PKG-1954852.pdf](https://www.mouser.com/datasheet/2/1127/APM_PSRAM_OPI_Xccela_APS6408L_OBMx_v3_5b_PKG-1954880.pdf)


### SOFTWARE:

#### MCU+SDK changes
  - Apply patch mcu_sdk_10_0_0_20_ospi_psram.patch from mcu sdk root folder, which does the following:
    - Added PSRAM to board lib
    - OSPI driver changes 
- Libs rebuild example:
  ```bash
  cd C:\ti\mcu_plus_sdk_am243x_10_00_00_20
  gmake libs PROFILE=debug
  ```
#### Notes:  
- Don't forget to update CCS version and other tools in `imports.mak`
- Fix for undeclared NULL error. Add in `psram.c`
  ```c
  #include <stddef.h>
  ```
- In case of errors building libs, you can use "libs-clean" and "libs-scrub" before retrying to build. Also, repeat for PROFILE=release if needed.

#### CCS Project   
- Import project in CCS. Then build project

#### Notes:
  - I2C:
    - I2C1 instance used.
    - Added "io_expander.c" to CCS project to configure IO expander by using TCA6424 APIs (`ioexp_tca6424.h`).
      - Used `TCA6424_Params_init()` which sets I2C address to 0x22
      - "ioIndex=19" which correspond to Pin23 (20)→ OSPI_SW_SEL.
          - LOW → OSPI FLASH
          - HIGH → OSPI_RAM
  - APS6408 Driver:
    - Header file for PSRAM configuration (`ospi_psram_aps6408.h`)
    - STIG Mode functions (`fss_config.h`)
    - `psram_ospi.c` implements:
      - `Psram_ospiRead()`, `Psram_ospiWrite()`, `Psram_ospiOpen()`, and `Psram_ospiClose()`
        - Write and read could be setup to DAC or INDAC modes.
  - `ospi_psram_io.c` implements a test application which writes, then read back and compares a buffer

### CONSOLE OUTPUT example:

  - [BOOTLOADER_PROFILE] Boot Media: NOR SPI FLASH
  - KPI_DATA: [BOOTLOADER_PROFILE] Boot Media Clock: 166.667 MHz
  - KPI_DATA: [BOOTLOADER_PROFILE] Boot Image Size: 0 KB
  - [BOOTLOADER_PROFILE] Cores present: r5f0-0
  - KPI_DATA: [BOOTLOADER PROFILE] SYSFW init: 11802us
  - KPI_DATA: [BOOTLOADER PROFILE] System_init: 364869us
  - KPI_DATA: [BOOTLOADER PROFILE] Drivers_open: 89us
  - KPI_DATA: [BOOTLOADER PROFILE] Board_driversOpen: 2523463us
  - KPI_DATA: [BOOTLOADER PROFILE] Sciclient Get Version: 13928us
  - KPI_DATA: [BOOTLOADER PROFILE] CPU load: 64385us
  - KPI_DATA: [BOOTLOADER PROFILE] SBL Total Time Taken: 2985155us

Image loading done, switching to application ...
PSRAM Manufacturer ID: 0xD
PSRAM Device ID: 0x2
All tests have passed!!

### OSPI PSRAM test:
- PSRAM WR/RD. Below combinations tested OK for 4KBytes
  - WRITE=INDAC; READ=DAC
  - WRITE=DAC; READ= DAC
  - WRITE=INDAC; READ=INDAC