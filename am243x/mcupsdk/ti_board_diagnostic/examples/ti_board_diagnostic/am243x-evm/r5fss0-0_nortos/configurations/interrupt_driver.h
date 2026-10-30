#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef INTERRUPT_DRIVER_H
#define INTERRUPT_DRIVER_H

#define INT0                 0U
#define INT1                 1U
#define INT2                 2U

#define MAX_INT_PIN    3U
#define ERROR_CODE     (-1)

typedef struct
{
    volatile bool intCallBackSet;
}INT_CALLBACK;

/* Assume MAX_INT_PIN and ISR callback array are defined */
extern void (*gpioIsrCallbacks[MAX_INT_PIN])(void *args);
extern HwiP_Object gGpioHwiObjects[MAX_INT_PIN];
extern INT_CALLBACK intCallBack[MAX_INT_PIN];

/* Define enums for trigger types and return codes */
typedef enum 
{
    TRIG_NONE = 0,
    TRIG_FALL_EDGE,
    TRIG_RISE_EDGE,
    TRIG_BOTH_EDGE,
} TriggerType;

typedef enum 
{
    SUCCESS = 0,
    INVALID_CONFIG_INTERRUPT = -1
} InterruptStatus;

extern InterruptStatus interruptConfig(uint8_t intPin, TriggerType triggType);

#endif
