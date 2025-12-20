# OSPI PSRAM (APS6408L) Driver and Example Code Analysis
## AM243x MCU+SDK Integration

---

## 1. Project Overview

This project implements an OSPI (Octal Serial Peripheral Interface) PSRAM driver for the **APS6408L** device (64 Mbit / 8 MB) integrated into the Texas Instruments **AM243x MCU+SDK**. The implementation provides both I/O mode access and Execute-In-Place (XIP) capability as examples, enabling the AM243x EVM to use external volatile PSRAM as additional high-speed memory.

**Key Capabilities:**
- Read/Write operations in DAC and INDAC modes
- Execute code directly from PSRAM (XIP)
- 8D-8D-8D DDR protocol support
- Mode register configuration
- Hardware IO expander control for FLASH/PSRAM selection

---

## 2. Directory Structure

```
am243x/examples/ospi/
├── ospi_psram_io/                    # I/O Mode Read/Write Test Example
│   ├── ospi_psram_io.c               # Main test application
│   ├── ospi_psram_aps6408.h          # APS6408L device definitions
│   ├── psram_ospi.h/.c               # PSRAM OSPI driver
│   ├── fss_config.h/.c               # FSS configuration (STIG mode)
│   ├── ti_board_psram_open_close.h/.c # Board-level PSRAM interface
│   ├── io_expander.c                 # I2C IO expander control
│   └── am243x-evm/
│       └── r5fss0-0_nortos/          # Build configuration for R5F core
│           ├── example.syscfg        # TI SysConfig file
│           ├── makefile*             # CCS build scripts
│           └── linker.cmd            # Linker script
│
└── ospi_psram_xip/                   # Execute-In-Place Example
    ├── ospi_psram_xip.c              # XIP demonstration
    ├── [Same driver files as above]
    └── am243x-evm/
        └── r5fss0-0_nortos/
```

---

## 3. Hardware Configuration

### 3.1 PSRAM Device Specifications
- **Part Number**: APS6408L-OBM-xBA (APM Semiconductor)
- **Memory Size**: 8 MB (64 Mbit)
- **Interface**: OSPI 8D-8D-8D (Octal DDR)
- **Base Address**: 0x60000000
- **Address Range**: 0x60000000 - 0x60800000
- **Vendor ID**: 0x0D
- **Device ID**: 0x02 (Generation 3)

### 3.2 AM243x EVM Configuration
- **Target Core**: R5FSS0-0 (Cortex-R5F)
- **OSPI Controller**: OSPI0 on CS1
- **OSPI Clock**: 200 MHz input clock
- **I2C Controller**: I2C1 for IO expander
- **IO Expander**: TCA6424 (I2C address 0x22)
- **OSPI_SW_SEL**: GPIO controlled via IO expander (pin 19) to switch between FLASH and PSRAM

### 3.3 Hardware Modifications

**Modified Hardware Components:**
- **AM243x EVM** - Base evaluation module
- **High-Speed Switch**: S3DDR3812RUAR - Multiplexes OSPI signals between FLASH and PSRAM
- **OSPI PSRAM Module**: APS6408L-OBM-BA device on custom PCB
- **Schematic Reference**: `PROC101E1_OSPI_SCH_AM243x.pdf`

**High-Speed Switch Configuration:**
- **Model**: S3DDR3812RUAR (high-speed multiplexer)
- **Control Signal**: OSPI_SW_SEL
  - **HIGH** = OSPI PSRAM selected (for PSRAM operations)
  - **LOW** = OSPI FLASH selected (default state)
- **Default State**: 10K pull-down resistor on OSPI_SW_SEL defaults to OSPI FLASH
- **Switching Control**: IO Expander (TCA6424) pin 19 (physical pin 23) drives OSPI_SW_SEL via I2C1

**Why High-Speed Switch is Required:**
- AM243x OSPI controller shares signals between FLASH and PSRAM
- Cannot access both devices simultaneously
- While two OSPI Chip Selects (CS0 for flash, CS1 for PSRAM) can theoretically operate without additional hardware, a high-speed switch is required in practice to prevent signal reflections on PCB traces connecting both devices, ensuring signal integrity at high speeds.

**Board Availability Note:**
This modified board configuration is a reference design and is not currently sold on TI.com. Users must implement similar hardware modifications to their AM243x EVM to enable OSPI PSRAM functionality.

---

## 4. Software Architecture

### 4.1 Driver Stack

```
Application Layer (ospi_psram_io.c / ospi_psram_xip.c)
        ↓
Board Interface Layer (ti_board_psram_open_close.c)
        ↓
PSRAM Driver Layer (psram_ospi.c)
        ↓
FSS Configuration Layer (fss_config.c - STIG mode)
        ↓
OSPI Hardware Driver (MCU+SDK OSPI driver)
        ↓
OSPI Hardware (OSPI0 controller)
```

### 4.2 Core Driver Files

#### `ospi_psram_aps6408.h` - Device Definitions
Defines APS6408L-specific constants:
- **Mode Registers**: MR0, MR1, MR2, MR4, MR6, MR8
- **Commands**: Read (0x20/0xA0), Write (0xA0), Reset (0xFF, 0x00), Mode Register Read/Write (0x40, 0xC0)
- **Memory Parameters**:
  - Size: 8 MB
  - Read Latency Code (RLC): 3 (3 cycles)
  - Write Latency Code (WLC): 3 (3 cycles)
  - Drive Strength: 3
  - Burst Type: 16-byte hybrid wrap

#### `psram_ospi.h/.c` - PSRAM Driver
**Data Structure:**
```c
typedef struct {
    OSPI_Handle ospiHandle;      // OSPI driver handle
    uint8_t dacEnable;           // Direct Access Controller enable
    uint8_t phyEnable;           // PHY mode enable
    uint8_t numAddrBytes;        // Address bytes (3 or 4)
    uint8_t badBlockCheck;       // Bad block checking
    uint8_t syncModeEnable;      // 1=INDAC, 0=DAC
} Psram_OspiObject;
```

**Key Functions:**
- `Psram_ospiOpen()` - Initialize PSRAM, configure OSPI controller, set device size, program mode registers
- `Psram_ospiRead()` - Read data from PSRAM (supports DAC/INDAC)
- `Psram_ospiWrite()` - Write data to PSRAM (supports DAC/INDAC)
- `Psram_ospiReadId()` - Read vendor/device ID via STIG mode
- `Psram_ospiReset()` - Reset PSRAM device
- `Psram_ospiReadCmd()` - Read mode registers
- `Psram_ospiWriteCmd()` - Write mode registers
- `Psram_ospiClose()` - Cleanup PSRAM driver

#### `fss_config.h/.c` - FSS Configuration
Low-level Flash SubSystem (FSS) configuration functions for STIG mode:
- `fss_set_dev_reg_enable()` - Enable device size register
- `fss_set_mem_bank_sz()` - Set memory bank size
- `fss_set_stig_cmd_ctrl_reg()` - Configure command control register
- `fss_set_stig_cmd_address()` - Set command address
- `fss_set_stig_cmd_opcode()` - Set command opcode
- `fss_set_stig_rd_wr_data()` - Configure read/write data
- `fss_set_dummy_cycle()` - Set dummy cycles for read/write
- `fss_enable_dac()` - Enable Direct Access Controller

**STIG Mode** (Software-controlled Triggered Instruction Generation):
- Direct register manipulation for command execution
- Used for initialization and mode register access
- Provides fine-grained control over OSPI transactions

#### `ti_board_psram_open_close.h/.c` - Board Interface
Board-level abstraction for PSRAM initialization:
- `Board_psramOpen()` - Open PSRAM driver, associate with OSPI0 handle
- `Board_psramClose()` - Close PSRAM driver
- Maintains PSRAM configuration and handle arrays
- Integrates with TI board infrastructure

#### `io_expander.c` - Hardware Multiplexer Control
- Configures TCA6424 I2C IO expander
- Sets OSPI_SW_SEL (pin 19) HIGH to select PSRAM
- I2C communication via I2C1 at address 0x22
- Required before PSRAM operations to route OSPI signals

---

## 5. Operation Modes

### 5.0 OSPI Controller Architecture

The OSPI module contains a **Data Target Controller** with two parallel access paths:

**Architecture Overview:**
- **DAC (Direct Access Controller)** - Memory-mapped access path
- **INDAC (Indirect Access Controller)** - Register-based access with internal SRAM buffer
- **Flash Command Generator** - Interfaces both controllers to the external PSRAM device

**DAC vs INDAC Operation:**

| Aspect | DAC (Direct Access) | INDAC (Indirect Access) |
|--------|---------------------|-------------------------|
| **Access Method** | Memory-mapped (CPU accesses PSRAM at 0x60000000 like internal SRAM) | Register-based (CPU configures transfer via OSPI registers) |
| **Data Path** | Direct CPU/DMA ↔ PSRAM | CPU/DMA ↔ OSPI Internal Buffer (FIFO) ↔ PSRAM |
| **Triggering** | Direct memory read/write operations | Software-triggered via Indirect Transfer registers |
| **Buffering** | None (direct) | Internal SRAM buffer with configurable thresholds |
| **CPU Involvement** | Continuous (every access) | Background transfer with interrupts when threshold reached |
| **Bus Overhead** | High (CPU waits for each access) | Low (CPU sets up transfer, handles buffered data via interrupts) |
| **Use Case** | Flash/PSRAM data transfers, executing code (XIP) | Processor-efficient bulk data transfers |

**INDAC Buffer Operation:**
- **Write**: Data sent by CPU is stored in internal buffer; transfer to PSRAM triggers when fill threshold is reached
- **Read**: Data read from PSRAM is stored in internal buffer; interrupt raised to CPU when configured byte count is received

### 5.1 DAC (Direct Access Controller) Mode
**Characteristics:**
- **Memory-mapped access** - PSRAM appears at 0x60000000, accessed like internal SRAM
- **No setup required** - Direct pointer operations: `data = *(uint32_t*)0x60000010;`
- **Standard functions work**: `memcpy()`, `memset()` operate without software triggering
- Uses `OSPI_readDirect()` / `OSPI_writeDirect()` APIs or direct memory access
- **Required for XIP** - CPU instruction fetch needs memory-mapped interface
- No buffering or transaction overhead

**Best For:** Single variables, small structs, random access, code execution (XIP)

### 5.2 INDAC (Indirect Access Controller) Mode
**Characteristics:**
- **Register-controlled transfers** - Configure once via OSPI registers, executes in background
- **Internal FIFO buffering** - OSPI SRAM buffer reduces bus overhead
- **Interrupt-driven** - CPU notified when thresholds met, can multitask during transfer
- Uses `OSPI_readIndirect()` / `OSPI_writeIndirect()` APIs
- Lower CPU involvement for large transfers

**Best For:** Large bulk transfers (multi-KB), streaming data, background operations where CPU efficiency matters

### 5.3 STIG (Software-Triggered Instruction Generation) Mode
**Characteristics:**
- Direct CSL register manipulation
- Used for special commands (reset, ID read, mode register access)
- Not for bulk data transfers
- Fine-grained control over OSPI protocol

**Use Cases:**
- PSRAM initialization
- Mode register configuration
- Device identification
- Special command sequences

---

## 6. Example Applications

### 6.1 ospi_psram_io.c - I/O Test Application

**Purpose**: Validate PSRAM read/write operations with data integrity verification

**Test Sequence:**
1. System initialization (`System_init()`)
2. Board drivers initialization (`Board_driversOpen()`)
3. IO expander configuration (`i2c_io_expander()`) - select PSRAM
4. PSRAM driver open (`Board_psramOpen()`)
5. Fill TX buffer with test pattern (0-1023)
6. Write data to PSRAM at offset 0 (DAC mode)
7. Read data from PSRAM at offset 0 (DAC mode)
8. Compare RX buffer with TX buffer
9. Report PASS/FAIL status
10. Cleanup and close

**Test Parameters:**
- Buffer size: 1024 bytes
- Test offset: 0x00000000
- Data pattern: Sequential 0-1023

**Verified Combinations:**
- ✓ WRITE=DAC, READ=DAC
- ✓ WRITE=INDAC, READ=DAC
- ✓ WRITE=INDAC, READ=INDAC

### 6.2 ospi_psram_xip.c - Execute-In-Place Application

**Purpose**: Demonstrate code execution directly from PSRAM memory

**XIP Process:**
1. Code marked with `__attribute__((section(".psram_code")))` initially placed in MSRAM (0x70140000)
2. After `Board_psramOpen()`, linker script copies code to PSRAM (0x60000000)
3. Application copies code back to different MSRAM location (0x70150000) for verification
4. Execute `testcode()` function pointer from PSRAM address
5. Verify execution completes successfully

**XIP Significance:**
- Tests PSRAM read reliability under instruction fetch conditions
- Validates memory-mapped execution

**Linker Configuration:**
- `.psram_code` section maps to 0x60000000
- XIP image generation via post-build scripts
- Boot image includes PSRAM code section

---

## 7. Initialization Sequence

### 7.1 Complete Boot Flow

```
1. System_init()
   └── Initialize system clocks, MPU, caches

2. Board_init()
   └── Initialize board infrastructure

3. Drivers_open()
   └── Open UART, I2C, OSPI drivers

4. i2c_io_expander()
   ├── Configure TCA6424 via I2C1
   └── Set OSPI_SW_SEL=HIGH (select PSRAM)

5. Board_driversOpen()
   └── Initialize OSPI0 hardware

6. Board_psramOpen()
   └── Psram_ospiOpen()
       ├── Configure OSPI controller for PSRAM
       ├── Set device size to 8MB
       ├── Reset PSRAM device
       ├── Read and verify device ID
       ├── Program mode registers (MR0, MR4, MR8)
       └── Enable DAC mode

7. Application code
   └── Read/Write operations or XIP execution
```

### 7.2 Mode Register Programming

During `Psram_ospiOpen()`, the following mode registers are configured:

- **MR0** (0x00000000): Read Latency Code = 3, Fixed latency, Reserved bits
- **MR4** (0x00000004): Write Latency Code = 3, Reserved bits
- **MR8** (0x00000008): Burst Type = 16-byte hybrid wrap, Deep Power Down disabled, Drive Strength = 3

---

## 8. MCU+SDK Integration & Build System

### 8.1 SDK Patch File: mcu_sdk_10_0_0_20_ospi_psram.patch

This patch is applied on top of **MCU+SDK 10.00.00.20** and extends the base SDK to add PSRAM support:

**Changes:**
1. **Board Library Makefile** (`source/board/makefile.am243x.r5f.ti-arm-clang.mk`):
   - Adds `psram.c` to SRCS_COMMON
   - Includes `psram.h` in build

2. **New File: psram.h** - Generic PSRAM Interface:
   ```c
   typedef struct Psram_Config_s {
       // PSRAM configuration structure
   } Psram_Config;

   typedef struct Psram_Fxns_t {
       int32_t (*openFxn)(Psram_Config *config, const Psram_Params *params);
       void (*closeFxn)(Psram_Config *config);
       int32_t (*readFxn)(Psram_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
       int32_t (*writeFxn)(Psram_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
   } Psram_Fxns;
   ```

### 8.2 SysConfig Integration (example.syscfg)

**Configured Modules:**
- **I2C1**: IO expander communication
  - Mode: Controller
  - Bitrate: 400 kHz (Fast Mode)

- **OSPI0**: PSRAM interface
  - Protocol: 8D-8D-8D
  - Chip Select: CS1
  - Input Clock: 200 MHz

- **UART0**: Debug console
  - Baudrate: 115200
  - Interrupt mode

- **Memory/MPU**:
  - MPU regions for PSRAM (0x60000000)
  - Cache policies

### 8.3 Makefile Structure

**Key Build Scripts:**
- `makefile` - Main build file
- `makefile_ccs_bootimage_gen` - Boot image generation
- `linker.cmd` - Memory layout and section placement

### 8.4 Boot Image Generation

**Process:**
1. Compile source files to object files
2. Link to create ELF executable
3. Generate RPRC (Resource Partition Resource Configuration) format
4. Create boot image with:
   - `appimage` - Standard application
   - `appimage_xip` - XIP-enabled application with PSRAM code at 0x60000000
5. Multi-core image generation (if applicable)
6. Boot signing for secure devices (HS/HS-FS)

**Post-Build Commands:**
```bash
# Generate RPRC
$(SYSCFG_NODE) $(SYSCFG_RPRC_CLI_PATH) $(APP_RPRC_SUFFIX) ...

# Generate boot image
$(BOOTIMAGE_GEN_CLI) --flash --offset ... --output $(BOOTIMAGE_NAME) ...

# Generate XIP boot image
$(BOOTIMAGE_GEN_CLI_XIP) --flash-xip --xip-start-addr 0x60000000 ...
```

### 8.5 Compiler and Toolchain

- **IDE**: Code Composer Studio (CCS)
- **Compiler**: TI ARM Clang (tiarmclang)
- **Target**: ARM Cortex-R5F
- **Optimization**: -Os (size optimization typical)
- **RTOS**: None (bare metal)
- **MCU+SDK Version**: 10.00.00.20

---

## 9. Memory Map

### 9.1 Address Space Layout

```
0x00000000 - 0x0007FFFF : R5F ATCM (512 KB) - Tightly coupled memory
0x00080000 - 0x0008FFFF : R5F BTCM (64 KB)
0x60000000 - 0x607FFFFF : OSPI PSRAM (8 MB) ← APS6408L mapped here
0x70000000 - 0x7017FFFF : MSRAM (1.5 MB) - Main shared RAM
0x...                   : OSPI FLASH (if selected)
```

### 9.2 PSRAM Region

- **Base**: 0x60000000
- **Size**: 0x800000 (8 MB)
- **End**: 0x607FFFFF
- **Access**: Read/Write/Execute
- **Cacheable**: Typically yes (depends on MPU config)

### 9.3 XIP Code Placement

For XIP applications:
- `.psram_code` section → 0x60000000
- Code copied from MSRAM to PSRAM during boot
- Executed directly from PSRAM via memory-mapped access

---

## 10. Performance Characteristics

### 10.1 OSPI Configuration

**Clock Configuration:**
- **Input Clock Frequency**: 200 MHz (to OSPI controller)
- **PHY Mode**: Disabled. PHY was not enabled due to project scope and resource availability
- **Effective OSPI Clock**: 200 MHz ÷ 4 = **50 MHz** (when PHY is disabled)
- **Protocol**: 8D-8D-8D (Octal DDR)
- **Performance**: 50 MHz × 2 (DDR) × 8 (octal) = **800 Mbps** (~100 MB/s theoretical)

**Latency Configuration:**
- **Read Latency**: 3 cycles (RLC=3)
- **Write Latency**: 3 cycles (WLC=3)
- **Dummy Cycles**: Read=5, Write=2

### 10.2 Access Modes Performance
- **DAC Mode**: Fastest, direct memory-mapped
- **INDAC Mode**: Slightly slower, transaction-based
- **STIG Mode**: Slowest, for special commands only

### 10.3 Use Case Recommendations
- **Bulk Data Buffers**: Use DAC mode
- **XIP Code**: Use DAC mode (memory-mapped)
- **Debugging**: Use INDAC mode
- **Initialization**: Use STIG mode

### 10.4 Benchmarking Best Practices

**Note**: Don't use `Psram_ospiRead()` or `Psram_ospiWrite()` for benchmarking as they add software overhead. Instead, use direct variable read/write or `memcpy()` to measure true hardware performance.

---

## 11. Testing and Verification

### 11.1 I/O Test Coverage
- Write/Read in DAC mode
- Write/Read in INDAC mode
- Mixed mode operations
- Data integrity verification (1024-byte buffer)
- Full offset range testing (implicit via sequential data)

### 11.2 XIP Test Coverage
- Code execution from PSRAM
- Code copy verification
- Memory-mapped instruction fetch
- Function pointer execution from PSRAM address

### 11.3 Device Verification
- Vendor ID check (0x0D)
- Device ID check (0x02)
- Mode register readback

---

## 12. Documentation and Resources

### 12.1 Project Documentation

**Hardware Documentation:**
- **[PROC101E1_OSPI_SCH_AM243x.pdf](https://github.com/TexasInstruments/Beyond-SDK/blob/main/am243x/examples/ospi/ospi_psram_io/am243x-evm/r5fss0-0_nortos/Docs/PROC101E1_OSPI_SCH_AM243x.pdf)** - Hardware schematic for AM243x EVM modifications
- **[APM_PSRAM_OPI_Xccela_APS6408L datasheet](https://www.mouser.com/datasheet/2/1127/APM_PSRAM_OPI_Xccela_APS6408L_OBMx_v3_5b_PKG-1954880.pdf)** - Device specifications

**Software Documentation:**
- **README.md** - Setup instructions, hardware modifications, test procedures

**Online Resources:**
- **[TI E2E FAQ: MCU-PLUS-SDK-AM243X OSPI PSRAM Memory Expansion](https://e2e.ti.com/f/1/t/1450933)**
- **[AM243x MCU+SDK User Guide](https://software-dl.ti.com/mcu-plus-sdk/esd/AM243X/latest/exports/docs/api_guide_am243x/GETTING_STARTED.html)**

---

## 13. Future Enhancements and Considerations

### 13.1 Performance Optimizations
- **PHY Enable Mode**: Implement PHY training and calibration sequences
  - Expected performance: ~400 MB/s (4x improvement over current 100 MB/s)
  - Requires additional initialization complexity
- **Performance Benchmarking**: Complete comprehensive benchmarking across all operation modes
  - DAC vs INDAC performance comparison
  - XIP execution performance metrics
- **Cache Optimization**: Fine-tune cache policies for PSRAM region

---

## 14. Summary

This project successfully integrates the **APS6408L OSPI PSRAM** (8 MB) into the AM243x MCU+SDK ecosystem, providing a solution for external volatile memory expansion.

**Key Achievements:**

- ✅ **Comprehensive Driver Stack** - Complete software architecture from application layer to hardware register level
- ✅ **Multiple Access Modes** - DAC (fast), INDAC (safe), and STIG (control) operation modes
- ✅ **Execute-In-Place (XIP) Support** - Execute code directly from PSRAM memory
- ✅ **Hardware Integration** - High-speed switch (S3DDR3812RUAR) for FLASH/PSRAM multiplexing via IO expander
- ✅ **Build System Integration** - CCS toolchain with boot image generation and XIP image support
- ✅ **Verified Operation** - I/O tests and XIP tests confirm functional correctness
