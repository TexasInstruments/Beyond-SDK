## Example Summary

This example demonstrates 4-wire resistive touch screen control using the MSPM0L1306 microcontroller.

### Available Examples

1. **resistive_touch_detection**: Detects touch events and measures X and Y coordinates once per touch.

### Hardware Requirements

These examples use a resistive panel with:
- X-axis: 780 Ohm resistance
- Y-axis: 250 Ohm resistance

For different panels, calculate current using V=IR:
1. Find resistance across each axis on your panel
2. Calculate current: I = V/R (assume 3.3V supply)
3. Each MSPM0L1306 pin supports maximum 6 mA

**Pin Count**: For Y-axis with 250Ω, 3 GPIO pins are required to avoid exceeding 6 mA per pin.

### Critical Design Note

Each axis must have **at least 1 ADC pin** (positive or negative):
- Required to read ADC values
- Used to detect touch presence
- Used to measure touch coordinates

## Pin Configurations

Visit [LP_MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Settings |
| --- | --- | --- | --- |
| PA17 | ADC12_0 | X+ | Connect to touch screen X+ |
| PA15 | ADC12_0 | X- | Connect to touch screen X- (ADC Channel 9) |
| PA16 | ADC12_0 | Y+ | Connect to touch screen Y+ (ADC Channel 8\) |
| PA24, PA0 | GPIO | Y+ additional | Additional Y+ connections |
| PA25, PA21, P22 | GPIO | Y- multiple | Multiple Y- connections |
| PA8 | UART0 | TX | UART TX (9600 baud) |
| PA9 | UART0 | RX | UART RX (9600 baud) |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0L1306 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0L1306 User's Guide](https://www.ti.com/lit/slau869).

## Example Usage

Compile, load and run the example. Open a UART terminal at 9600 baud to view touch data.

### UART Output Format

At startup:
- Prints "DEBUG: Touch detection started"

When touch is detected:
- Initial message: "TOUCH DETECTED!"
- For simple touch detection example: "TOUCH DETECTED! X=XXXX  Y=YYYY"

### How It Works

1. **Touch Detection**: Voltage applied across Y+ and Y- pins. ADC measures voltage at X+ pin. Touch detected when ADC value exceeds 4000.
2. **Coordinate Measurement**:
   - X-coordinate: Voltage applied to X+ and X- pins. ADC reads from Y+ pin (channel 8).
   - Y-coordinate: Voltage applied to Y+ and Y- pins. ADC reads from X+ pin (channel 9).
3. **Touch Classification for Simple Touch Detection example**:
   - No touch classification. Only detects and measures touch coordinates once.
   - Outputs coordinates when touch is detected.

### Configuration Parameters

| Parameter | Value | Location | Description |
| --- | --- | --- | --- |
| X_MIN | 200 | main.c:39 | Minimum X-coordinate for valid touch (calibration dependent) |
| X_MAX | 4090 | main.c:40 | Maximum X-coordinate for valid touch (calibration dependent) |
| Y_MIN | 200 | main.c:41 | Minimum Y-coordinate for valid touch (calibration dependent) |
| Y_MAX | 3800 | main.c:42 | Maximum Y-coordinate for valid touch (calibration dependent) |
| INACTIVE_SAMPLE_DELAY | 200 | main.c:47 | Sampling rate in ms when no touch detected |
| SETTLE_DELAY | 3.125 | main.c:51 | Voltage settling time in ms after pin configuration |
| TOUCH_DETECTION_THD | 4000 | resistive_detection.h:40 | ADC threshold for touch detection (12-bit resolution) |
| X_ADC_CHANNEL | 9 | resistive_detection.h:36 | ADC channel for X+ pin (used for Y-coordinate reading) |
| Y_ADC_CHANNEL | 8 | resistive_detection.h:37 | ADC channel for Y+ pin (used for X-coordinate reading) |

### Hardware Connections

Connect the 4-wire resistive touch screen to:
- X+ → PA17
- X- → PA15 (ADC channel 9)
- Y+ → PA16 (ADC channel 8\) with additional connections to PA24 and PA0
- Y- → PA21, PA22 and PA25

