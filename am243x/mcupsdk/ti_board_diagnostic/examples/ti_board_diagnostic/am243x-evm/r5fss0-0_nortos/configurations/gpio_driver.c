/**
 * @file gpio_driver.c
 * @brief GPIO driver implementation for TI AM2432 Board Diagnostic project
 * @author Tushar Thakur
 * 
 * This file contains the GPIO driver implementation for the Board Diagnostic UART COM Ports
 * communication project. It provides GPIO pin configuration, read/write operations,
 * pulse generation, and utility functions for both MCU and SoC GPIO domains.
 */

#include "../header_files/common_extern.h"
#include "../configurations/gpio_driver.h"
#include "../header_files/uart_cmd_header.h"

/* System clock frequency for timing calculations (25 MHz) */
const uint32_t F_HZ   = 25000000UL;

/* Pinmux configuration arrays for GPIO pins */
Pinmux_PerCfg_t pMux_GPIO[2];

/* Base addresses for MCU and SoC GPIO domains */
uint32_t baseAddGPIO_MCU,baseAddGPIO_SoC;

/* Buffer for storing GPIO read operations */
uint8_t readGPIOBuff[GPIO_MAX_INDEX];

/* Pin offset mapping for GPIO pin multiplexing configuration */
const uint16_t pGPIO_Offset[8] = {
    PIN_MCU_SPI1_CS0, PIN_MCU_UART1_RTSN, PIN_PRG0_PRU0_GPO0, PIN_PRG0_PRU0_GPO1, PIN_PRG0_PRU1_GPO0, PIN_SPI1_D1, PIN_PRG0_PRU0_GPO2,
    PIN_PRG0_PRU0_GPO3
};

/* GPIO pin ID mapping for hardware register access */
const uint32_t gpioPinID[8] = {
    GPIO0_PIN, GPIO1_PIN, GPIO2_PIN, GPIO3_PIN, GPIO4_PIN, GPIO5_PIN, GPIO6_PIN,
    GPIO7_PIN
};

/**
 * @brief Configure GPIO pin mode and direction
 * @param pin GPIO pin number (0-7)
 * @param mode GPIO mode (input, output, pull-up, pull-down)
 * @return 0 on success, -1 on error
 * 
 * Configures the specified GPIO pin with the desired mode and direction.
 * Handles both MCU and SoC GPIO domains with appropriate pinmux configuration.
 */
int32_t GPIO_Set(uint8_t pin, GPIO_Mode mode)
{
    uint32_t settings = PIN_MODE(7);  /* GPIO mode 7 for general purpose I/O */
    uint32_t direction;

    /* Validate pin range */
    if (pin >= GPIO_MAX_INDEX) 
    {
        return -1;
    }

    /* Configure pinmux register for GPIO functionality */
    pMux_GPIO[0].offset = pGPIO_Offset[pin];
    pMux_GPIO[1].offset = PINMUX_END;
    pMux_GPIO[1].settings = PINMUX_END;

    /* Configure GPIO mode settings */
    switch (mode) 
    {
        case GPIO_MODE_NO_CHANGE:
            return 0;

        case GPIO_MODE_OUTPUT:
            settings = (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE);  /* Disable pull resistors for output */
            direction = GPIO_DIRECTION_OUTPUT;
            break;

        case GPIO_MODE_INPUT:
            settings = (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE);  /* Enable input buffer */
            direction = GPIO_DIRECTION_INPUT;
            break;

        case GPIO_MODE_INPUT_PULL_UP:
            settings = (PIN_MODE(7) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION);  /* Input with pull-up */
            direction = GPIO_DIRECTION_INPUT;
            break;

        case GPIO_MODE_INPUT_PULL_DOWN:
            /* Assuming PIN_PULL_DOWN handled by default or another flag */
            settings = (PIN_MODE(7) | PIN_INPUT_ENABLE);
            direction = GPIO_DIRECTION_INPUT;
            break;

        default:
            return -1; /* Invalid mode */
    }

    pMux_GPIO[0].settings = settings;

    /* Determine GPIO domain and configure accordingly */
    if (pin < 2) 
    {
        /* MCU GPIO domain (pins 0-1) */
        baseAddGPIO_MCU = (uint32_t)AddrTranslateP_getLocalAddr(CSL_MCU_GPIO0_BASE);
        Pinmux_config(pMux_GPIO, PINMUX_DOMAIN_ID_MCU);
        GPIO_setDirMode(baseAddGPIO_MCU, gpioPinID[pin], direction);
        DebugP_logInfo("Configured MCU_GPIO_PIN_%d with %s and direction %s.\r\n", gpioPinID[pin], (mode == GPIO_MODE_OUTPUT) ? "RX Active & Pull Disable" : (mode == GPIO_MODE_INPUT) ? "Rx Active & Pull Disable" : (mode == GPIO_MODE_INPUT_PULL_UP) ? "Rx Active & INPUT_PULL_UP" : (mode == GPIO_MODE_INPUT_PULL_DOWN) ? "Rx Active & INPUT_PULL_DOWN" : "NO_CHANGE", (direction == GPIO_DIRECTION_OUTPUT) ? "OUTPUT" : "INPUT");
    } 
    else 
    {
        /* SoC GPIO domain (pins 2-7) */
        baseAddGPIO_SoC = (uint32_t)AddrTranslateP_getLocalAddr(CSL_GPIO1_BASE);
        Pinmux_config(pMux_GPIO, PINMUX_DOMAIN_ID_MAIN);
        GPIO_setDirMode(baseAddGPIO_SoC, gpioPinID[pin], direction);
        DebugP_logInfo("Configured GPIO1_PIN_%d with %s and direction %s.\r\n", gpioPinID[pin], (mode == GPIO_MODE_OUTPUT) ? "Rx Active & Pull Disable" : (mode == GPIO_MODE_INPUT) ? "Rx Active & Pull Disable" : (mode == GPIO_MODE_INPUT_PULL_UP) ? "Rx Active & INPUT_PULL_UP" : (mode == GPIO_MODE_INPUT_PULL_DOWN) ? "Rx Active & INPUT_PULL_DOWN" : "NO_CHANGE", (direction == GPIO_DIRECTION_OUTPUT) ? "OUTPUT" : "INPUT");
    }
    return 0;
}

/**
 * @brief Write GPIO pin state (high or low)
 * @param pin GPIO pin number (0-7)
 * @param state GPIO state (high, low, or no change)
 * @return 0 on success, -1 on error
 * 
 * Sets the output state of the specified GPIO pin. Automatically handles
 * both MCU and SoC GPIO domains based on pin number.
 */
int32_t GPIO_Write(uint8_t pin, GPIO_State state)
{
    /* Validate base addresses */
    if (!(baseAddGPIO_MCU || baseAddGPIO_SoC)) 
    {
        return -1; // GPIO not initialized
    }

    /* Validate pin range */
    if (pin >= GPIO_MAX_INDEX) 
    {
        return -1; // Invalid pin
    }

    /* Determine base address based on pin number */
    uint32_t baseAddr = (pin < 2) ? baseAddGPIO_MCU : baseAddGPIO_SoC;

    /* Set GPIO pin state */
    switch (state) 
    {
        case GPIO_STATE_NO_CHANGE:
            break;

        case GPIO_STATE_LOW:
            GPIO_pinWriteLow(baseAddr, gpioPinID[pin]);
            DebugP_logInfo("%s_PIN_%d set to LOW.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[pin]);
            break;

        case GPIO_STATE_HIGH:
            GPIO_pinWriteHigh(baseAddr, gpioPinID[pin]);
            DebugP_logInfo("%s_PIN_%d set to HIGH.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[pin]);
            break;

        default:
            return -1; // Invalid state
    }

    return 0;
}

/**
 * @brief Read GPIO pin state
 * @param pin GPIO pin number (0-7)
 * @return GPIO_READ_LOW, GPIO_READ_HIGH, or GPIO_READ_ERROR
 * 
 * Reads the current state of the specified GPIO pin. Handles both MCU
 * and SoC GPIO domains automatically.
 */
GPIO_ReadValue GPIO_Read(int8_t pin)
{
    /* Validate pin range */
    if (pin < 0 || pin >= GPIO_MAX_INDEX) 
    {
        return GPIO_READ_ERROR; // Invalid pin
    }

    /* Validate base addresses */
    if (!(baseAddGPIO_MCU || baseAddGPIO_SoC)) 
    {
        return GPIO_READ_ERROR; // GPIO not initialized
    }

    /* Determine base address based on pin number */
    uint32_t baseAddr = (pin < 2) ? baseAddGPIO_MCU : baseAddGPIO_SoC;

    /* Read pin state and convert to standard return values */
    int8_t value = GPIO_pinRead(baseAddr, gpioPinID[pin]);
    DebugP_logInfo("%s_PIN_%d read as %s.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[pin], (value == 0) ? "LOW" : "HIGH");
    return (value == 0) ? GPIO_READ_LOW : GPIO_READ_HIGH;
}

/**
 * @brief Microsecond delay using general purpose timer
 * @param usec Delay duration in microseconds
 * @return 0 on success
 * 
 * Provides precise microsecond delays using the general purpose timer.
 * Handles timer wraparound and uses busy-wait for short delays.
 */
int32_t delay_us(uint32_t usec)
{
    /* Convert microseconds to timer ticks (round up to never under-deliver) */
    const uint32_t ticks = (uint32_t)((((uint64_t)usec * F_HZ + 999999ULL) / 1000000ULL));
    const uint32_t start = GPTIMER_getCount(gGpTimerHandle[3]);
    
    /* Wait for specified ticks (wraparound-safe with unsigned subtraction) */
    while ((GPTIMER_getCount(gGpTimerHandle[3]) - start) < ticks) 
    {
        /* Busy-wait loop for microsecond-scale delays */
    }
    
    return 0;
}

/**
 * @brief Generate pulse on GPIO pin with specified polarity and duration
 * @param _pin GPIO pin number (0-7)
 * @param polarity Pulse polarity (low-first or high-first)
 * @param duration Pulse duration in microseconds
 * @return 0 on success, -1 on error
 * 
 * Generates a controlled pulse on the specified GPIO pin. The pin is
 * automatically configured as output and toggled with precise timing.
 */
int8_t GPIO_WritePulse(uint8_t _pin, GPIO_PulsePolarity polarity, uint16_t duration)
{
    /* Validate pin range */
    if (_pin < 0 || _pin >= GPIO_MAX_INDEX)
    {
        return -1; // Invalid pin
    }

    /* Validate duration (minimum 5 microseconds for reliable operation) */
    if((duration != 0) && (duration < 5))
    {
        return -1;
    }
    
    /* Ensure pin is configured as output */
    GPIO_Set(_pin,GPIO_MODE_OUTPUT);

    /* Determine base address based on pin number */
    uint32_t baseAddr = (_pin < 2) ? baseAddGPIO_MCU : baseAddGPIO_SoC;
    
    /* Read current output register value */
    uint8_t portVal = GPIO_pinOutValueRead(baseAddr, gpioPinID[_pin]);
    DebugP_logInfo("%s_PIN_%d initial state as %s.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin], (portVal == 0) ? "LOW" : "HIGH");

    /* Set initial state opposite to current state for clean pulse */
    if(portVal == 0)
    {
        GPIO_pinWriteHigh(baseAddr, gpioPinID[_pin]); 
    }
    else
    {
        GPIO_pinWriteLow(baseAddr, gpioPinID[_pin]);
    }
    DebugP_logInfo("%s_PIN_%d state changed to %s.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin], (portVal == 1) ? "LOW" : "HIGH");
    
    /* Generate pulse with specified polarity */
    switch (polarity) 
    {
        case GPIO_PULSE_LOW_FIRST:
            GPIO_pinWriteLow(baseAddr, gpioPinID[_pin]);
            DebugP_logInfo("%s_PIN_%d state changed to LOW.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin]);
            delay_us(duration);
            GPIO_pinWriteHigh(baseAddr, gpioPinID[_pin]);
            DebugP_logInfo("%s_PIN_%d state changed to HIGH.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin]);
            break;

        case GPIO_PULSE_HIGH_FIRST:
            GPIO_pinWriteHigh(baseAddr, gpioPinID[_pin]);
            DebugP_logInfo("%s_PIN_%d state changed to HIGH.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin]);
            delay_us(duration);
            GPIO_pinWriteLow(baseAddr, gpioPinID[_pin]);
            DebugP_logInfo("%s_PIN_%d state changed to LOW.\r\n", (baseAddr == baseAddGPIO_MCU) ? "MCU_GPIO" : "GPIO1" , gpioPinID[_pin]);
            break;

        default:
            return -1; /* Invalid polarity */
    }
    return 0;
}
