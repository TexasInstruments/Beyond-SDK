#include "../header_files/common_extern.h"
#include "gpio_driver.h"
#include "interrupt_driver.h"
#include <drivers/sciclient.h>
#include <string.h>

#define INVALID_CONFIG_INTERRUPT            (-1)

#define MAX_EVENT_COUNTS                    (65535U)

/* Interrupt Pin Index from GPIO PinID Buffer */
const uint32_t intPinID[3] = {GPIO3_PIN,GPIO4_PIN,GPIO5_PIN};
const uint32_t intPinIndex[3] = {3,4,5};
const uint32_t intRouterID[3] = {GPIO3_INTR_NUM,GPIO4_INTR_NUM,GPIO5_INTR_NUM};

HwiP_Object gGpioHwiObjects[MAX_INT_PIN];
INT_CALLBACK intCallBack[MAX_INT_PIN] = {{FALSE},{FALSE},{FALSE}};

extern uint32_t baseAddGPIO_MCU,baseAddGPIO_SoC;
/*
 * Board info
*/

/** \brief bank interrupt source index base */
#define TISCI_BANK_SRC_IDX_BASE_WKUP_GPIO0  (128U)
#define TISCI_BANK_SRC_IDX_BASE_GPIO0       (90U)
#define TISCI_BANK_SRC_IDX_BASE_GPIO1       (90U)
#define TISCI_BANK_SRC_IDX_BASE_MCU_GPIO0   (30U)

int32_t Sciclient_gpioIrqSet(uint16_t pinInt, uint16_t intRouteId);
int32_t Sciclient_gpioIrqRelease(uint16_t pinInt, uint16_t intRouteId);
void INT0_ISR_CALLBACK(void *args);
void INT1_ISR_CALLBACK(void *args);
void INT2_ISR_CALLBACK(void *args);

/* Array of ISR callback function pointers */
void (*gpioIsrCallbacks[MAX_INT_PIN])(void *args) = {
    INT0_ISR_CALLBACK,
    INT1_ISR_CALLBACK,
    INT2_ISR_CALLBACK
};


/*  ---------------------------------------- ISR CALLBACKS ------------------------- */

#include <stdint.h>

volatile int32_t _INT_EventCout[MAX_INT_PIN] = {0,0,0};
uint8_t interruptSendFlage = 0;

static void UARTWriteCMDIRQ(void)
{
    uint8_t writeBufUART[10];

    /* SIGNATURE */
    writeBufUART[0] = 0x54; // 'T'
    writeBufUART[1] = 0x49; // 'I'
    /* SEQUENCE NUMBER - 2BYTES */
    writeBufUART[2] = 00;
    writeBufUART[3] = 00;
    /* RESERVED BYTES - 2BYTES */
    writeBufUART[4] = 0;
    writeBufUART[5] = 0;
    /* FUNCTION CODE - 1BYTE */
    writeBufUART[6] = 04;
    /* COMMAND CODE - 1BYTE */
    writeBufUART[7] = 0xF1;
    /* DATA LEN - 1BYTE */
    writeBufUART[8] = 6;
    /* STAUTS CODE - 1BYTE */
    writeBufUART[9] = 0;

    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf = writeBufUART;
    trans.count = 10;
    if(UART_write(gUartHandle[CONFIG_UART1], &trans))
    {
        DebugP_logError("UART WRITE OPERATION FAIL..\n");
    }
}

void IRQ_Packet_Transfer(void)
{
    if(interruptSendFlage == 1)
    {
        interruptSendFlage = 0;
        UARTWriteCMDIRQ();
        SendISRPackets(INT0,_INT_EventCout[INT0]);
    }
    else if(((interruptSendFlage >> 1)&0x01)== 1)
    {
        interruptSendFlage = 0;
        UARTWriteCMDIRQ();
        SendISRPackets(INT1,_INT_EventCout[INT1]);
    }
    else if(((interruptSendFlage >> 2)&0x01)== 1)
    {
        interruptSendFlage = 0;
        UARTWriteCMDIRQ();
        SendISRPackets(INT2,_INT_EventCout[INT2]);
    }
}
/* INT0 ISR Callback */
void INT0_ISR_CALLBACK(void *args)
{
    uint32_t pinNum0 = (uint32_t) args;
    uint32_t intrStatus0;

    if (baseAddGPIO_SoC)
    {
        intrStatus0 = GPIO_getBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum0));
        GPIO_clearBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum0), intrStatus0);

        if (intrStatus0 & GPIO_GET_BANK_BIT_MASK(pinNum0))
        {
            /* Increment event counter with wrap-around */
            _INT_EventCout[INT0] = (_INT_EventCout[INT0] < UINT32_MAX) ? ++_INT_EventCout[INT0] : 0;
            if(intCallBack[INT0].intCallBackSet)
            {
                interruptSendFlage = 1;
            }
        }
    }
}

/* INT1 ISR Callback */
void INT1_ISR_CALLBACK(void *args)
{
    uint32_t pinNum1 = (uint32_t) args;
    uint32_t intrStatus1;

    if (baseAddGPIO_SoC)
    {
        intrStatus1 = GPIO_getBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum1));
        GPIO_clearBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum1), intrStatus1);
        if (intrStatus1 & GPIO_GET_BANK_BIT_MASK(pinNum1))
        {
            _INT_EventCout[INT1] = (_INT_EventCout[INT1] < UINT16_MAX) ? ++_INT_EventCout[INT1] : 0;
            if(intCallBack[INT1].intCallBackSet)
            {
                interruptSendFlage |= (1 << 1) & 0x0F;
            }
        }
    }
}

/* INT2 ISR Callback */
void INT2_ISR_CALLBACK(void *args)
{
    uint32_t pinNum2 = (uint32_t) args;
    uint32_t intrStatus;

    if (baseAddGPIO_SoC)
    {
        intrStatus = GPIO_getBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum2));
        GPIO_clearBankIntrStatus(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(pinNum2), GPIO_GET_BANK_BIT_MASK(pinNum2));
        if (intrStatus & GPIO_GET_BANK_BIT_MASK(pinNum2))
        {
            _INT_EventCout[INT2] = (_INT_EventCout[INT2] < UINT16_MAX) ? ++_INT_EventCout[INT2] : 0;
            if(intCallBack[INT2].intCallBackSet)
            {
                interruptSendFlage |= (1 << 2) & 0x0F;
            }
           
        }
    }
}

int32_t Sciclient_gpioIrqSet(uint16_t pinInt, uint16_t intRouteId)
{
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = TISCI_DEV_GPIO1;
    rmIrqReq.src_index              = pinInt;
    rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
    rmIrqReq.dst_host_irq           = intRouteId;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    return(Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER));
}

int32_t Sciclient_gpioIrqRelease(uint16_t pinInt, uint16_t intRouteId)
{
    struct tisci_msg_rm_irq_release_req rmIrqReq;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = TISCI_DEV_GPIO1;
    rmIrqReq.src_index              = pinInt;
    rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
    rmIrqReq.dst_host_irq           = intRouteId;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    return (Sciclient_rmIrqRelease(&rmIrqReq, SystemP_WAIT_FOREVER));
}


int32_t Reg_INT(uint8_t _intPin)
{
    HwiP_Params hwPrmsINT;
    /* Initialize interrupt parameters */
    HwiP_Params_init(&hwPrmsINT);
    hwPrmsINT.intNum = intRouterID[_intPin];
    hwPrmsINT.callback = gpioIsrCallbacks[_intPin]; // Use array for callbacks
    hwPrmsINT.args = (void *)intPinID[_intPin];
    hwPrmsINT.isPulse = TRUE;
    hwPrmsINT.priority = 6;
    /* Construct interrupt object */
    return HwiP_construct(&gGpioHwiObjects[_intPin], &hwPrmsINT);
}

InterruptStatus interruptConfig(uint8_t intPin,TriggerType triggType)
{
    uint8_t triggerState = GPIO_TRIG_TYPE_NONE;  // Default trigger type
    bool enableInterrupt = true;

    /* Validate intPin range */
    if (intPin >= MAX_INT_PIN) 
    {
        return INVALID_CONFIG_INTERRUPT;
    }

    /* Determine trigger type */
    switch (triggType) 
    {
        case TRIG_NONE:
            enableInterrupt = false;
            break;
        case TRIG_FALL_EDGE:
            triggerState = GPIO_TRIG_TYPE_FALL_EDGE;
            break;
        case TRIG_RISE_EDGE:
            triggerState = GPIO_TRIG_TYPE_RISE_EDGE;
            break;
        case TRIG_BOTH_EDGE:
            triggerState = GPIO_TRIG_TYPE_BOTH_EDGE;
            break;
        default:
            return INVALID_CONFIG_INTERRUPT;
    }

    if (enableInterrupt) 
    {
        /* Configure and enable interrupt */
        GPIO_Set(intPinIndex[intPin], GPIO_MODE_OUTPUT);
        if (triggType != TRIG_NONE) 
        {
            GPIO_setTrigType(baseAddGPIO_SoC, intPinID[intPin], triggerState);
            DebugP_logInfo("Trigger Type set to %s for GPIO1_PIN_%d.\r\n", (triggerState == GPIO_TRIG_TYPE_FALL_EDGE) ? "FALL_EDGE" : (triggerState == GPIO_TRIG_TYPE_RISE_EDGE) ? "RISE_EDGE" :    "BOTH_EDGE", intPinID[intPin]);
        }
        GPIO_bankIntrEnable(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(intPinID[intPin]));
        uint32_t status = Sciclient_gpioIrqSet(intPinID[intPin], intRouterID[intPin]);
        if(status) 
        {
            DebugP_logError("Interrupt Routing Failed for GPIO1_PIN_%d.\r\n", intPin);
            return INVALID_CONFIG_INTERRUPT;
        }
        status = Reg_INT(intPin);
        if(status)
        {
            DebugP_logError("Interrupt Registration Failed for GPIO1_PIN_%d.\r\n", intPin);
            return INVALID_CONFIG_INTERRUPT;
        }
    } 
    else 
    {
        /* Disable interrupt */
        GPIO_bankIntrDisable(baseAddGPIO_SoC, GPIO_GET_BANK_INDEX(intPinID[intPin]));
        GPIO_setTrigType(baseAddGPIO_SoC, intPinID[intPin], GPIO_TRIG_TYPE_NONE);
        GPIO_clearIntrStatus(baseAddGPIO_SoC, intPinID[intPin]);
        Sciclient_gpioIrqRelease(intPinID[intPin], intRouterID[intPin]);
    }
    return SUCCESS;
}
