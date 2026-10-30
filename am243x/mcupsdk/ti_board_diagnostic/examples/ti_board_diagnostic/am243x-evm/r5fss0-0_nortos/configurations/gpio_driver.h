#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#define GPIO_MAX_INDEX              8U

typedef enum 
{
        GPIO_MODE_NO_CHANGE      = 0,  /* No change */
        GPIO_MODE_OUTPUT         = 1,  /* Output mode */
        GPIO_MODE_INPUT          = 2,  /* Input mode without pull */
        GPIO_MODE_INPUT_PULL_UP  = 3,  /* Input mode with pull-up */
        GPIO_MODE_INPUT_PULL_DOWN= 4   /* Input mode with pull-down */
} GPIO_Mode;

typedef enum 
{
    GPIO_STATE_NO_CHANGE = 0,  /* No change */
    GPIO_STATE_LOW       = 1,  /* Drive pin LOW */
    GPIO_STATE_HIGH      = 2   /* Drive pin HIGH */
} GPIO_State;

typedef enum 
{
    GPIO_PULSE_LOW_FIRST = 0,  /* Pulse starts LOW then goes HIGH */
    GPIO_PULSE_HIGH_FIRST = 1  /* Pulse starts HIGH then goes LOW */
} GPIO_PulsePolarity;

typedef enum 
{
    GPIO_READ_ERROR = -1,  /* Error or invalid pin */
    GPIO_READ_LOW   = 0,   /* Pin is LOW */
    GPIO_READ_HIGH  = 1    /* Pin is HIGH */
} GPIO_ReadValue;

/*
 * GPIO
 */
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>

/* GPIO PIN Macros */
#define GPIO0_BASE_ADDR (CSL_MCU_GPIO0_BASE)
#define GPIO0_PIN (5)
#define GPIO0_DIR (GPIO_DIRECTION_INPUT)
#define GPIO0_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO0_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_0)

#define GPIO1_BASE_ADDR (CSL_MCU_GPIO0_BASE)
#define GPIO1_PIN (17)
#define GPIO1_DIR (GPIO_DIRECTION_INPUT)
#define GPIO1_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO1_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_1)

#define GPIO2_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO2_PIN (0)
#define GPIO2_DIR (GPIO_DIRECTION_INPUT)
#define GPIO2_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO2_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_2)

#define GPIO3_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO3_PIN (1)
#define GPIO3_DIR (GPIO_DIRECTION_INPUT)
#define GPIO3_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO3_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_3)

#define GPIO4_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO4_PIN (20)
#define GPIO4_DIR (GPIO_DIRECTION_INPUT)
#define GPIO4_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO4_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_4)

#define GPIO5_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO5_PIN (51)
#define GPIO5_DIR (GPIO_DIRECTION_INPUT)
#define GPIO5_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO5_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_5)

#define GPIO6_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO6_PIN (2)
#define GPIO6_DIR (GPIO_DIRECTION_INPUT)
#define GPIO6_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO6_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_6)

#define GPIO7_BASE_ADDR (CSL_GPIO1_BASE)
#define GPIO7_PIN (3)
#define GPIO7_DIR (GPIO_DIRECTION_INPUT)
#define GPIO7_TRIG_TYPE (GPIO_TRIG_TYPE_RISE_EDGE)

#define GPIO7_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_7)

#define GPIO_PIN_MAX                  8U


/* Extern */
extern int32_t GPIO_Set(uint8_t pin, GPIO_Mode mode);
extern int32_t GPIO_Write(uint8_t pin, GPIO_State state);
extern int8_t GPIO_ReadStateBuff(uint8_t tBytes, uint8_t* rxGPIOData);
extern int8_t GPIO_Read(int8_t pin);
extern int8_t GPIO_WritePulse(uint8_t pin, uint8_t polartiy, uint16_t duration);
#endif // GPIO_DRIVER_H
