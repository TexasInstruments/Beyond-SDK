# Overview

## Multi Byte Single Channel

This directory contains SPI controller applications for AM62L processors. These communicate with MSPM0 peripherals to receive ADC data over SPI.

### Files

- `spi_multi_byte_single_channel_multiple_txns.c` - Multiple transaction mode
- `spi_multi_byte_single_channel_single_txns.c` - Single transaction mode

### Single Transaction Mode

Uses 16-bit word transfers to receive ADC data.

**Configuration:**
- Device: `/dev/spidev1.0`
- Bits per word: 16
- Speed: 500 kHz (default)
- Data format: Single 16-bit transaction
- Default TX command: `0x0F35`

**Usage:**
```bash
./spi_multi_byte_single_channel_single_txns
```

The application reads ADC data from MSPM0 in a single 16-bit SPI transaction.
Data is received as a 16-bit word and displayed as two separate byte values.

### Multiple Transaction Mode

Uses 8-bit word transfers to receive ADC data across two sequential bytes.

**Configuration:**
- Device: `/dev/spidev3.0`
- Bits per word: 8
- Speed: 500 kHz (default)
- Data format: Two sequential 8-bit transactions
- Default TX command: `0x00, 0x00`

**Usage:**
```bash
./spi_multi_byte_single_channel_multiple_txns
```

**Important:** For multiple transaction mode, the MSPM0 configuration file must be modified. The ADC must send 8 bits in a frame instead of 12 bits used in single transaction mode.

The application continuously reads ADC data and combines the two bytes into a single value.

### Command Line Options

Both multi-byte applications support these options:

- `-D --device` - SPI device path
- `-s --speed` - Max speed in Hz
- `-b --bpw` - Bits per word
- `-d --delay` - Delay in microseconds
- `-v --verbose` - Enable verbose output
- `-H --cpha` - Set clock phase
- `-O --cpol` - Set clock polarity
- `-L --lsb` - LSB first transmission
- `-C --cs-high` - Chip select active high
- `-N --no-cs` - No chip select
- `-i --input` - Input data from file
- `-o --output` - Output data to file

### Data Reception

**Single Transaction:**
Receives data in one 16-bit transfer from `spi_multi_byte_single_channel_single_txns.c:167`.
Output format: `Data = rx[0], rx[1]` (two separate byte values)

**Multiple Transactions:**
Receives data as two 8-bit bytes from `spi_multi_byte_single_channel_multiple_txns.c:169-170`.
Data is reconstructed as a 16-bit value:
```c
data = (rx[0] << 8) | rx[1]
```
Output format: `Data = XXXX` (combined 16-bit value)

### Building

Cross-compile for AM62L:
```bash
${CC} -o spi_multi_byte_single_channel_single_txns spi_multi_byte_single_channel_single_txns.c
${CC} -o spi_multi_byte_single_channel_multiple_txns spi_multi_byte_single_channel_multiple_txns.c
```

Where `${CC}` is your cross-compiler.

## Single Byte Single Channel

This directory contains SPI controller application for AM62L to communicate with MSPM0 peripherals for receiving single-byte ADC data.

### File

- `spi_single_byte_single_channel.c` - Single byte transfer mode

### Configuration

Uses 8-bit word transfers to receive ADC data in a single byte.

**Configuration:**
- Device: `/dev/spidev1.0`
- Bits per word: 8
- Speed: 500 kHz (default)
- Data format: Single 8-bit transaction
- Default TX command: `0x35`

**Usage:**
```bash
./spi_single_byte_single_channel
```

The application continuously reads 8-bit ADC data from MSPM0.

### Command Line Options

Supports standard SPI options:

- `-D --device` - SPI device path
- `-s --speed` - Max speed in Hz
- `-b --bpw` - Bits per word
- `-v --verbose` - Enable verbose output
- `-H --cpha` - Set clock phase
- `-O --cpol` - Set clock polarity
- `-d --delay` - Delay in microseconds

### Data Reception

Receives ADC data as single 8-bit value.
Data is printed as: `Data = rx[0], rx[1]`

### Building

Cross-compile for AM62L:
```bash
${CC} -o spi_single_byte_single_channel spi_single_byte_single_channel.c
```

Where `${CC}` is your cross-compiler.

