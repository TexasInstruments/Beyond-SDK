# Fast Serial Interface (FSI) Technology Guide

## 1. FSI Technology Overview

### What is FSI?

FSI is a high-speed serial communication peripheral supporting up to **200 Mbps** using minimal signals. It enables MCU communication on shared voltage planes, across isolation barriers (e.g., ISO77xx), and in distributed control systems requiring low latency.

### Key Technical Characteristics

**Physical Layer:**
- Up to 3 signal lines per direction: 1 clock + up to 2 data lines
- Dual-lane operation can achieve maximum 200 Mbps at 50 MHz clock
- Data transmitted on both clock edges

**Frame Structure:**
- Programmable data length up to 16 words per frame (each word is 16 bits, maximum 256 bits or 32 bytes per frame)
- Hardware-managed CRC for error detection
- ECC support for enhanced reliability
- Frame types: data frames, ping frames, error frames

**Architecture:**
- Independent FSITX (transmitter) and FSIRX (receiver) cores
- No master/slave concept - full simultaneous bidirectional communication
- Minimum 4 signal lines needed for 2-way point-to-point communication

**Advanced Features:**
- Delay line skew compensation and line-break detection (PING/Frame watchdog)
- DMA support and hardware pass-through mode for minimal latency
- Sub-100 ns event synchronization across devices

**Benefits Over Alternative Protocols:**
Compared to traditional protocols (CAN, SPI, UART), FSI delivers higher data rates (up to 200 Mbps), lower latency, and simplified isolation barrier design with minimal signal lines.

## 2. Common Applications

FSI is designed for industrial systems requiring fast, low-latency, synchronized communication between multiple devices in distributed control architectures.

### Industrial Applications

**Power Module Systems:**
Parallel power module architectures in industrial drives, telecom rectifiers, server power supplies, and on-board chargers require fast, synchronized communication to exchange:
- Protection signals (fault conditions, safety events)
- Sampling parameters (ADC coordination)
- Control loop data (setpoints, feedback values)

### Key Application Examples

**C2000 Multi-Axis Servo Drives:** Device nodes control individual axes with FSI providing the communication link for control loop information, enabling precise motion control coordination. See [TIDM-02006 reference design](https://www.ti.com/tool/TIDM-02006).

**C2000 Decentralized Power Control:** Wide bandgap devices (GaN, SiC) enable sophisticated power distribution architectures where FSI connects distributed MCUs for coordinated control. See [application report](https://www.ti.com/lit/pdf/spracy4).

### Network Topologies

#### **Daisy-Chain (Ring) Topology**

**Configuration:**
- Devices connected in series forming a ring
- Each device has 1 FSI TX module and 1 FSI RX module
- Data forwarded through intermediate nodes: Device 1 TX → Device 2 RX, Device 2 TX → Device 3 RX, ..., Device N TX → Device 1 RX (completing the ring)

```
  ┌───────────┐         ┌───────────┐                 ┌───────────┐
  │ Device 1  │         │ Device 2  │                 │ Device N  │
  │           │         │           │                 │           │
  │  ┌────┐   │         │  ┌────┐   │                 │  ┌────┐   │
  │  │ TX │───┼────────→│  │ RX │   │   ────...───    │  │ TX │───┼──┐
  │  └────┘   │         │  └────┘   │                 │  └────┘   │  │
  │           │         │           │                 │           │  │
  │  ┌────┐   │         │  ┌────┐   │                 │  ┌────┐   │  │
  │  │ RX │   │         │  │ TX │───┼────────  ...  ──┼─→│ RX │   │  │
  │  └─┬──┘   │         │  └────┘   │                 │  └────┘   │  │
  │    │      │         │           │                 │           │  │
  └────┼──────┘         └───────────┘                 └───────────┘  │
       │                                                             │
       └──────────────────────── Loop back ──────────────────────────┘
```
#### **Star Topology**

**Configuration:**
- Central host device connects directly to multiple node devices
- Host TX broadcasts to all node RX modules (shown with ═══)
- Each node TX connects to independent host RX module (shown with ───)

```
  ┌──────────────┐                                ┌──────────────┐
  │      TX      │════════════════════════════╗══→│     RX       │
  │              │                            ║   │              │
  │    RX_1      │←───────────────────────────║───│     TX       │
  │              │                            ║   │              │
  │    RX_2      │←─────────────┐             ║   │   Node 1     │
  │              │              │             ║   └──────────────┘
  │     ...      │              │             ║
  │              │              │             ║
  │    RX_N      │←─────┐       │             ║   ┌──────────────┐
  │              │      │       │             ╠══→│     RX       │
  │ Host Device  │      │       └─────────────╬───│     TX       │
  └──────────────┘      │                     ║   │              │
                        │                     ║   │   Node 2     │
                        │                     ║   └──────────────┘
                        │                     ║
                        │                     ║        ...
                        │                     ║
                        │                     ║   ┌──────────────┐
                        │                     ╠══→│     RX       │
                        │                     ║   │              │
                        └─────────────────────╬───│     TX       │
                                                  │              │
                                                  │   Node N     │
                                                  └──────────────┘



Legend:
  ═══→  Broadcast connections (Host TX to all Node RX modules)
  ───   Point-to-point connections (Each Node TX to dedicated Host RX)
```
**Example**: F2838x family has 2 FSI TX + 8 FSI RX modules, supporting up to 8 nodes per transmitter.

## 3. FSI Hardware Connection Reference Guide

This guide provides physical hardware connection details for testing FSI communication on AM243x platforms.

---

### Single-Board External Loopback

#### AM243x LaunchPad (LP)

**Hardware Requirements:**
- 1× AM243x LaunchPad (LAUNCHXL-AM243X)
- 2× Jumper wires

**Pin Connections:**

| Connection | From Pin | To Pin | Description |
|------------|----------|--------|-------------|
| Clock | J16.1 (TX0_CLK) | J16.2 (RX0_CLK) | FSI clock loopback |
| Data | J16.5 (TX0_D0) | J16.6 (RX0_D0) | FSI data line loopback |

**Software Configuration Note:**
- No I2C IO Expander configuration needed (hardware mux defaults to FSI via pull-down resistor)

**References:**
- [AM243x LP User's Guide](https://www.ti.com/lit/ug/spruj12f/spruj12f.pdf) - Section 4.14 (FSI Interface) for official pin mapping
- [E2E FAQ - AM243x Single-Board FSI Loopback](https://e2e.ti.com/support/processors/f/791/t/1374607) - Setup instructions and troubleshooting

---

#### AM243x EVM

**Hardware Requirements:**
- 1× AM243x EVM (TMDS243EVM)
- 2× Jumper wires

**Pin Connections (Rev C boards use J7):**

| Connection | From Pin | To Pin | Description |
|------------|----------|--------|-------------|
| Clock | J7.1 (TX0_CLK) | J7.2 (RX0_CLK) | FSI clock loopback |
| Data | J7.5 (RX0_D0) | J7.6 (TX0_D0) | FSI data line loopback |

**Important - Software Configuration Required:**
- **I2C IO Expander configuration required** to route FSI signals through hardware mux (TCA6424A controls TS3A27518E to select FSI over GPMC)
- See Section 5 for code example: `i2c_io_expander()`

**Board Revision Note:** Earlier revisions use J5 instead of J7. Verify your board revision.

**References:**
- [AM243x EVM User's Guide](https://www.ti.com/lit/ug/spruj63b/spruj63b.pdf) - Section 3.4.20 (FSI Interface) for official pin mapping
- [E2E FAQ - AM243x Single-Board FSI Loopback](https://e2e.ti.com/support/processors/f/791/t/1374607) - Setup instructions and troubleshooting

---

### Multi-Board FSI Communication

#### Two AM243x LaunchPads

**Hardware Requirements:**
- 2× AM243x LaunchPads (LAUNCHXL-AM243X)
- 5× Jumper wires
- USB cables for UART console (both boards)

**Pin Connections (via J16 headers):**

| Signal | Device 1 (Lead) Pin | Device 2 (Node) Pin | Description |
|--------|---------------------|---------------------|-------------|
| TX Clock → RX Clock | FSI_TX0_CLK (J16.2) | FSI_RX0_CLK (J16.1) | Lead TX clock to Node RX clock |
| RX Clock ← TX Clock | FSI_RX0_CLK (J16.1) | FSI_TX0_CLK (J16.2) | Node TX clock to Lead RX clock |
| TX Data → RX Data | FSI_TX0_D0 (J16.6) | FSI_RX0_D0 (J16.5) | Lead TX data to Node RX data |
| RX Data ← TX Data | FSI_RX0_D0 (J16.5) | FSI_TX0_D0 (J16.6) | Node TX data to Lead RX data |
| Ground | GND (J16.3/J16.4) | GND (J16.3/J16.4) | Common ground |

**Reference:** [FAQ - 2-Device FSI Loopback](https://e2e.ti.com/support/processors/f/791/t/1374617)

---

#### AM243x EVM and LaunchPad

**Hardware Requirements:**
- 1× AM243x EVM (TMDS243EVM)
- 1× AM243x LaunchPad (LAUNCHXL-AM243X)
- 5× Jumper wires
- USB cables for UART console (both boards)

**Pin Connections:**

| Signal | Device 1 (EVM) Pin | Device 2 (LP) Pin | Description |
|--------|-------------------|-------------------|-------------|
| TX Clock → RX Clock | FSI_TX0_CLK (J7.1) | FSI_RX0_CLK (J16.1) | EVM TX clock to LP RX clock |
| RX Clock ← TX Clock | FSI_RX0_CLK (J7.2) | FSI_TX0_CLK (J16.2) | LP TX clock to EVM RX clock |
| TX Data → RX Data | FSI_TX0_D0 (J7.5) | FSI_RX0_D0 (J16.5) | EVM TX data to LP RX data |
| RX Data ← TX Data | FSI_RX0_D0 (J7.6) | FSI_TX0_D0 (J16.6) | LP TX data to EVM RX data |
| Ground | GND (J7.3/J7.4) | GND (J16.3/J16.4) | Common ground |

**Reference:** [FAQ - 2-Device FSI Loopback](https://e2e.ti.com/support/processors/f/791/t/1374617)

---

### Recommended: TMDSFSIADAPEVM Adapter Board

For multi-device FSI networks, the TMDSFSIADAPEVM adapter board simplifies connections and provides integrated isolation.

**Key Features:**
- **Simple Wiring**: Use standard CAT5 Ethernet cables instead of individual jumper wires
- **Built-in Isolation**: ISO7763 digital isolator and LVDS/RS-485 transceivers on-board
- **Multi-Topology**: Supports daisy-chain, star, and custom network configurations
- **Plug-and-Play**: Compatible with F28002x, F28004x, F2838x LaunchPads and EVMs

**Connection Example:**
```
LaunchPad 1       TMDSFSIADAPEVM       CAT5 Cable       TMDSFSIADAPEVM       LaunchPad 2
  (Lead)    ←→    Adapter Board    ←→  (RJ45)    ←→    Adapter Board    ←→   (Node)
```

**Resources:**
- Order: [TMDSFSIADAPEVM](https://www.ti.com/tool/TMDSFSIADAPEVM)
- Documentation: [User's Guide (SPRUIO9)](https://www.ti.com/lit/pdf/spruio9)

---

### Troubleshooting Tips

**No Communication:**
- Verify all ground connections are secure
- Confirm TX/RX clock and data lines are cross-connected correctly (TX → RX, RX ← TX)
- Check cable integrity and proper seating of connectors

**Intermittent Errors:**
- Reduce cable length or use shielded cables to minimize signal degradation and EMI
- Verify signal integrity with oscilloscope (check for noise, ringing, overshoot)
- **AM243x EVM**: Always use short ribbon cables for board-to-board or board-to-adapter connections
- Consider TMDSFSIADAPEVM adapter for electrical isolation, longer cable runs, or high data rates

## 4. Implementation Guidance

### Lead-Node Architecture Pattern

FSI implementations typically use a **Lead-Node (D1-D2) architecture**:

**Lead Device (D1):**
- Initiates communication handshake
- Transmits data frames
- Acts as timing master for synchronization

**Node Devices (D2, D3, ...)**
- Respond to handshake from Lead
- Receive data and process/forward as needed
- Follow synchronization signals from Lead

### Protocol Flow and Handshaking

**Initial Setup:**
1. **I/O Configuration**: Configure pins for FSI routing (device-specific)
2. **FSI Initialization**: Configure TX and RX base addresses and parameters
3. **Interrupt Setup**: Register callback functions for TX/RX events
4. **Handshake Execution**: Establish communication link

**Handshake Sequence (Daisy-Chain/Ring Topology):**

The handshake mechanism prepares each device in the chain before actual data transmission, accommodating scenarios where devices may power up in arbitrary order. Two ping loops are required:

*Ping Loop 0 - Establish Communication Path:*

**Purpose**: Verify all devices in the ring are powered and ready for reception.

1. **Lead device**:
   - Sends flush sequence + PING frame with Tag0
   - Enters wait loop for RX interrupt
   - If valid Ping Tag0 received (confirms path established), proceed to Ping Loop 1
   - Otherwise, retry Ping Loop 0

2. **Node devices** (sequential propagation):
   - Enter wait loop for RX interrupt
   - Upon receiving valid Ping Tag0 from previous device, forward flush sequence + Ping Tag0 to next device
   - If invalid or no Ping Tag0 received, retry Ping Loop 0

3. **Result**: Ping Tag0 propagates through Device 2 → Device 3 → ... → Device N → back to Lead (ring closure)

*Ping Loop 1 - Acknowledge Ready State:*

**Purpose**: Inform all nodes that communication path is validated and to begin expecting data.

1. **Lead device**:
   - Sends PING frame with Tag1
   - Waits to receive Ping Tag1 back through ring
   - Upon receiving Tag1, handshake completes

2. **Node devices**:
   - Receive Ping Tag1 from previous device
   - Forward Ping Tag1 to next device
   - Transition to data-ready state

3. **Result**: All devices confirmed ready for data transmission

**Implementation Functions:**
- `handshake_lead()`: Lead device handshake logic (fsi_loopback_interrupt_d1.c:421-485)
- `handshake_node()`: Node device handshake logic (fsi_loopback_interrupt_d2.c)

**Handshake Sequence (Star Topology):**

A similar two-loop handshake sequence applies to star topology, with key difference being **broadcast** vs. sequential propagation:

*Ping Loop 0 - Establish All Paths:*

1. **Host device**:
   - Broadcasts flush sequence + PING frame with Tag0 to all nodes simultaneously
   - Waits to receive Ping Tag0 from **all nodes** on independent RX channels (RX_1, RX_2, ..., RX_N)
   - Once all nodes respond with Tag0, proceed to Ping Loop 1

2. **Node devices** (parallel reception):
   - Each node receives broadcast Ping Tag0 on its RX
   - Each node responds by sending Ping Tag0 back to host on its dedicated TX → Host RX channel
   - Nodes operate independently (no forwarding between nodes)

*Ping Loop 1 - Acknowledge Ready State:*

1. **Host device**:
   - Broadcasts PING frame with Tag1 to all nodes
   - Waits to receive Ping Tag1 from all nodes
   - Upon receiving Tag1 from all nodes, handshake completes

2. **Node devices**:
   - Receive broadcast Ping Tag1
   - Respond with Ping Tag1 back to host
   - Transition to data-ready state

**Key Difference**: Star uses broadcast (one-to-many) with parallel responses, while daisy-chain uses sequential forwarding through ring.

**Data Transfer Phase:**
- Data frames (up to 16 words) exchanged
- Interrupt-driven or DMA-based operation
- Loopback verification for communication integrity (optional during initial development)

### Development Best Practices

**Hardware Initialization Order:**
```c
// Always follow this sequence:
1. Configure I/O routing (if needed)
2. initFSI(txBase, rxBase)
3. Setup interrupts and callbacks
4. Execute handshake (lead or node specific)
5. Begin data transfer
```

**Key Configuration Parameters:**
- `FSI_APP_TXCLK_FREQ`: TX clock frequency (1-50 MHz typical)
- `FSI_APP_FRAME_DATA_WORD_SIZE`: Words per frame (max 16)
- `FSI_APP_N_LANES`: Single (0) or dual (1) lane mode
- Frame tags and user data: Application-specific identifiers

**Debugging Strategies:**

*Console Verification:*
- Lead should show transmission/reception counters
- Node should show matching receive/forward counts
- Both should confirm "All tests have passed"

**Skew Compensation:**

FSI's programmable RX delay lines compensate for clock-to-data skew from PCB traces and isolators. SDK calibration algorithms maintain 200 Mbps operation in systems with isolation or long traces.

## 5. AM243x Code Examples

This section provides practical implementation details for FSI on AM243x platforms, including initialization sequences, driver APIs, and common data transfer patterns.

### Code Structure

The AM243x examples demonstrate FSI in bare-metal (NORTOS) mode with two implementation types:

**FSI Loopback (`fsi_loopback/`):**
- Basic interrupt-driven communication between Lead (D1) and Node (D2) devices
- Lead transmits 100 frames of test data, Node echoes back for verification
- Demonstrates core FSI initialization, handshaking, and data transfer patterns

**FSI Secondary Bootloader (`fsi_sbl/`):**
- Advanced firmware-over-FSI implementation
- Lead (D1) reads application image from SD card (`/sd0/app`) and transmits via FSI
- Node (D2) receives app chunks, validates with MAGIC_WORD protocol, executes received code
- Demonstrates chunked transfer (32-byte frames) and protocol design

**Directory Organization:**
```
fsi/
├── fsi_loopback/
│   ├── fsi_loopback_interrupt_d1_am243x-evm_r5fss0-0_nortos_ti-arm-clang/
│   │   ├── fsi_loopback_interrupt_d1.c    # Lead implementation
│   │   ├── main.c                         # Entry point
│   │   └── example.syscfg                 # Hardware config
│   └── fsi_loopback_interrupt_d2_am243x-evm_r5fss0-0_nortos_ti-arm-clang/
│       ├── fsi_loopback_interrupt_d2.c    # Node implementation
│       └── ...
└── fsi_sbl/
    ├── sbl_fsi_d1_am243x-evm_r5fss0-0_nortos_ti-arm-clang/
    │   └── sbl_fsi_d1.c                   # SD card reader + FSI transmit
    └── sbl_fsi_d2_am243x-evm_r5fss0-0_nortos_ti-arm-clang/
        └── sbl_fsi_d2.c                   # FSI receive + app execution
```

### Initialization Sequence

**Hardware Setup:**
- Two AM243x devices (EVM-to-EVM, LP-to-LP, or EVM-to-LP)
- FSI TX0 → RX0 connections via J7 (EVM) or J16 (LP) headers
- See Section 3 for complete wiring details and pin mappings
- 500 MHz input clock, prescaled to 1 MHz (loopback) or 40 MHz (SBL)

**Important - Platform-Specific Requirements:**
- **AM243x EVM**: I2C IO Expander configuration **required** (TCA6424A controls mux to select FSI over GPMC)
- **AM243x LaunchPad**: No configuration needed (hardware mux defaults to FSI)

**Initialization Flow:**
```c
// 1. System and board initialization
System_init();
Board_init();
Drivers_open();
Board_driversOpen();

// 2. Configure I2C IO Expander to route FSI signals (EVM ONLY)
//    EVM: TCA6424A controls TS3A27518E mux to select FSI over GPMC
//    LaunchPad: Not needed (TMUX154EDGSR mux defaults to FSI via hardware pull-down)
i2c_io_expander();  // EVM: Set TCA6424 Pin8 (P07) HIGH to select FSI

// 3. Initialize FSI hardware
initFSI(CONFIG_FSI_TX0_BASE_ADDR, CONFIG_FSI_RX0_BASE_ADDR);
    // - Disable internal loopback: FSI_disableRxInternalLoopback()
    // - Configure TX: FSI_performTxInitialization(txBase, prescaler)
    // - Configure RX: FSI_performRxInitialization(rxBase)

// 4. Setup interrupt handling
Fsi_appIntrInit(txBaseAddr, rxBaseAddr);
    // - Create binary semaphores for TX/RX synchronization
    // - Register HwiP callbacks for TX_FRAME_DONE and RX_DATA_FRAME events
    // - Enable FSI interrupts

// 5. Execute handshake protocol
handshake_lead(txBaseAddr, rxBaseAddr);  // Lead device
// or
handshake_node(txBaseAddr, rxBaseAddr);  // Node device

// 6. Configure frame parameters
Fsi_appTxConfig(txBaseAddr);  // Frame type, size, lanes, tag, user data
Fsi_appRxConfig(rxBaseAddr);  // Frame size, lanes
```

### Key Driver APIs

**Initialization APIs:**
```c
FSI_performTxInitialization(baseAddr, prescaler);  // Initialize TX with clock prescaler
FSI_performRxInitialization(baseAddr);             // Initialize RX module
FSI_disableRxInternalLoopback(baseAddr);           // Disable loopback mode
```

**Configuration APIs:**
```c
// Frame Configuration
FSI_setTxFrameType(baseAddr, FSI_FRAME_TYPE_NWORD_DATA);  // or FSI_FRAME_TYPE_PING
FSI_setTxSoftwareFrameSize(baseAddr, nWords);             // 1-16 words per frame
FSI_setTxDataWidth(baseAddr, FSI_DATA_WIDTH_1_LANE);      // Single lane (or 0x0U)
FSI_setTxUserDefinedData(baseAddr, userData);             // 8-bit user data field
FSI_setTxFrameTag(baseAddr, tag);                         // 4-bit frame identifier (0-15)
FSI_setTxBufferPtr(baseAddr, bufIdx);                     // Buffer index (0-15)

// RX Configuration (mirror TX settings)
FSI_setRxSoftwareFrameSize(baseAddr, nWords);
FSI_setRxDataWidth(baseAddr, FSI_DATA_WIDTH_1_LANE);      // Single lane (or 0x0U for single, 0x1U for dual)
FSI_setRxBufferPtr(baseAddr, bufIdx);
```

**Data Transfer APIs:**
```c
// Transmit
FSI_writeTxBuffer(baseAddr, data[], nWords, bufIdx);  // Write to TX buffer
FSI_startTxTransmit(baseAddr);                        // Trigger transmission
FSI_executeTxFlushSequence(baseAddr, prescaler);     // Send FLUSH sync signal

// Receive
FSI_readRxBuffer(baseAddr, data[], nWords, bufIdx);  // Read from RX buffer

// Status and Frame Info (pass variable pointer to receive value)
FSI_getRxFrameType(baseAddr, &type);                 // Get received frame type
FSI_getRxFrameTag(baseAddr, &tag);                   // Get received frame tag
FSI_getRxPingTag(baseAddr, &tag);                    // Get PING frame tag
```

**Interrupt Management:**
```c
// Enable interrupts
FSI_enableTxInterrupt(baseAddr, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
FSI_enableRxInterrupt(baseAddr, FSI_INT1, FSI_RX_EVT_DATA_FRAME);

// In ISR: Get and clear events
FSI_getTxEventStatus(baseAddr, &status);
FSI_clearTxEvents(baseAddr, FSI_TX_EVTMASK);
FSI_getRxEventStatus(baseAddr, &status);
FSI_clearRxEvents(baseAddr, status);
```

### Data Transfer Patterns

**Loopback Transfer (100 iterations):**
```c
// Initialize loop counter
uint32_t loopCnt = FSI_APP_LOOP_COUNT;

while(loopCnt--) {
    // 1. Prepare TX buffer with test pattern
    for(uint32_t i = 0; i < 16; i++) {
        gTxBufData[i] = loopCnt + i;  // Incrementing test data
    }

    // 2. Transmit frame
    FSI_writeTxBuffer(txBaseAddr, gTxBufData, 16, 0);
    FSI_startTxTransmit(txBaseAddr);

    // 3. Wait for TX/RX completion (interrupt-driven)
    SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);

    // 4. Read and verify received data
    FSI_readRxBuffer(rxBaseAddr, gRxBufData, 16, 0);
    Fsi_appCompareData(gTxBufData, gRxBufData, loopCnt);
}
```

**Handshake Pattern (Lead Device):**
```c
// Stage 1: Establish communication path (Ping Loop 0)
while(1) {
    FSI_executeTxFlushSequence(txBaseAddr, prescaler);
    FSI_setTxFrameTag(txBaseAddr, FSI_FRAME_TAG0);
    FSI_setTxFrameType(txBaseAddr, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(txBaseAddr);

    if(SemaphoreP_pend(&gFsiRxSemObject, 10) == SystemP_SUCCESS) {
        uint16_t pingTag;
        FSI_getRxPingTag(rxBaseAddr, &pingTag);
        if(pingTag == FSI_FRAME_TAG0) break;  // Path confirmed
    }
}

// Stage 2: Acknowledge readiness (Ping Loop 1)
while(1) {
    FSI_setTxFrameTag(txBaseAddr, FSI_FRAME_TAG1);
    FSI_startTxTransmit(txBaseAddr);

    if(SemaphoreP_pend(&gFsiRxSemObject, 10) == SystemP_SUCCESS) {
        uint16_t pingTag;
        FSI_getRxPingTag(rxBaseAddr, &pingTag);
        if(pingTag == FSI_FRAME_TAG1) break;  // Ready for data
    }
}
```

**SBL Transfer Pattern (Firmware Update):**
```c
// 1. Send MAGIC_WORD_START + file size (4 words)
gTxBufData[0] = 0xFEED;  // APP_MAGIC_WORD_START = 0xC0DEFEED
gTxBufData[1] = 0xC0DE;
gTxBufData[2] = fileSize & 0xFFFF;
gTxBufData[3] = (fileSize >> 16) & 0xFFFF;
FSI_writeTxBuffer(txBaseAddr, gTxBufData, 4, 0);
FSI_startTxTransmit(txBaseAddr);

// 2. Transmit app image in 32-byte chunks (16 words)
uint32_t dataframeBytes = 32;  // 16 words * 2 bytes
uint32_t loopCnt = fileSize / dataframeBytes;
uint32_t remainderBytes = fileSize % dataframeBytes;
uint32_t totalFrames = loopCnt + ((remainderBytes > 0) ? 1 : 0);
uint32_t appImageOffset = 0;

for(uint32_t i = 0; i < totalFrames; i++) {
    uint16_t currWords;
    uint16_t currBytes;

    if(i < loopCnt) {
        currWords = 16;             // Full frame
        currBytes = dataframeBytes;
    } else {
        // Last frame: handle remainder
        currWords = remainderBytes / 2;
        if(remainderBytes % 2 != 0) currWords++;  // Round up
        currBytes = remainderBytes;
    }

    memcpy(gTxBufData, &gAppImageBuf[appImageOffset], currBytes);
    appImageOffset += currBytes;

    FSI_writeTxBuffer(txBaseAddr, gTxBufData, currWords, 0);
    FSI_startTxTransmit(txBaseAddr);

    SemaphoreP_pend(&gFsiTxSemObject, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);  // Wait for echo

    // Verify echoed data
    FSI_readRxBuffer(rxBaseAddr, gRxBufData, currWords, 0);
    Fsi_appCompareData(gTxBufData, gRxBufData, i);
}

// 3. Wait for MAGIC_WORD_END confirmation (0xCAFEBABE)
SemaphoreP_pend(&gFsiRxSemObject, SystemP_WAIT_FOREVER);
FSI_readRxBuffer(rxBaseAddr, gRxBufData, 2, 0);

// Verify MAGIC_WORD_END (high 16 bits in gRxBufData[0], low 16 bits in gRxBufData[1])
if ((gRxBufData[0] != ((APP_MAGIC_WORD_END >> 16) & 0xFFFF)) ||
    (gRxBufData[1] != (APP_MAGIC_WORD_END & 0xFFFF)))
{
    DebugP_log("[FSI] MAGIC_WORD_END mismatch\r\n");
    status = SystemP_FAILURE;
}
else
{
    DebugP_log("[FSI] Application transfer complete\r\n");
}
```

**Note:** This is a simplified example for clarity. The actual SBL implementation includes:
- **Timeout and retry logic**: Uses `SemaphoreP_pend(&gFsiRxSemObject, 10)` with timeout to detect communication failures and automatically retransmit lost frames
- **MAGIC_WORD_END verification**: Both Lead and Node verify the completion magic word (0xCAFEBABE) with error checking
- **SD card integration**: Lead device reads application image from `/sd0/app` file before transmission
- **Robust error handling**: Validates frame sizes, handles partial transfers, and reports detailed error messages

### Interrupt Handling

**Semaphore-Based Synchronization:**
```c
// Binary semaphores created at init (count = 0)
SemaphoreP_constructBinary(&gFsiTxSemObject, 0);
SemaphoreP_constructBinary(&gFsiRxSemObject, 0);

// HwiP callback posts semaphore when event occurs
static void Fsi_appTxCallback(void *args) {
    FSI_getTxEventStatus(txBaseAddr, &txEventSts);
    FSI_clearTxEvents(txBaseAddr, FSI_TX_EVTMASK);
    SemaphoreP_post(&gFsiTxSemObject);  // Unblock waiting thread
}

static void Fsi_appRxCallback(void *args) {
    FSI_getRxEventStatus(rxBaseAddr, &rxEventSts);
    FSI_clearRxEvents(rxBaseAddr, rxEventSts);
    SemaphoreP_post(&gFsiRxSemObject);  // Unblock waiting thread
}
```

**Note:** These are simplified callback examples for clarity. The actual implementation extracts `txBaseAddr`/`rxBaseAddr` from the `args` parameter and may include additional status flags like `fsiTxInt1Received` or `fsiRxInt1Received`.

## 6. Build System

**TI Ecosystem Tools:**
- Code Composer Studio (CCS) IDE for development and debugging
- TI ARM Clang toolchain (`ti-arm-clang`)
- SysConfig generates hardware initialization code (`ti_drivers_config.c/h`)
- MCU+ SDK v10.1.0.32+ provides FSI drivers and board support

**Key Configuration Constants:**
```c
// From example.syscfg
#define CONFIG_FSI_TX0_BASE_ADDR    0x...      // TX module base
#define CONFIG_FSI_RX0_BASE_ADDR    0x...      // RX module base
#define CONFIG_FSI_TX0_CLK          500000000  // 500 MHz input clock

// Application defines
#define FSI_APP_TXCLK_FREQ          1000000    // 1 MHz (loopback) or 40 MHz (SBL)
#define FSI_APP_FRAME_DATA_WORD_SIZE    16     // Max words per frame
#define FSI_APP_LOOP_COUNT              100    // Test iterations
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   0x20000  // 128 KB (SBL)
```

## 7. Summary

FSI provides high-speed, low-latency communication in distributed control systems:

- **Speed**: Up to 200 Mbps with minimal signal count
- **Reliability**: Hardware CRC, watchdog, and error detection
- **Flexibility**: Multiple topologies, triggering modes, and frame configurations
- **Synchronization**: Sub-100 ns event alignment across isolation barriers
- **Scalability**: From simple 2-device to complex multi-node networks

---

**Additional Resources:**
- [Application note](https://www.ti.com/lit/an/spracm3e/spracm3e.pdf)
