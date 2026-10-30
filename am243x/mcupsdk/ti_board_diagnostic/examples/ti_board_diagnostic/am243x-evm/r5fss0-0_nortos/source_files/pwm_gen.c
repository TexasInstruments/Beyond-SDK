/**
 * @file PWM_Gen.c
 * @brief PWM generation implementation for TI AM2432 Board Diagnostic project
 * @author Tushar Thakur
 * @version 1.0
 * @date 2026
 * 
 * This file implements PWM (Pulse Width Modulation) generation using GPTIMER
 * peripherals. It provides functions to configure, enable, and disable PWM
 * outputs with configurable frequency and duty cycle.
 */

#include "../header_files/common_extern.h"
#include "../header_files/uart_cmd_header.h"
#include "../configurations/gpio_driver.h"

/*----------------------- PWM CHANNELS ------------------------------- */
/**
 * @brief PWM Channels
 */
#define PWM0                                  0U
#define PWM1                                  1U
#define PWM2                                  2U


/* Timer overflow value for 32-bit timer */
#define PWM_TIMER_OVERFLOW                    0xFFFFFFFE

/* Physical pin mapping for PWM outputs */
const uint16_t pPWM_Offset[3] = {PIN_I2C1_SCL, PIN_I2C1_SDA, PIN_PRG1_PRU0_GPO17};

/* External GPTIMER configuration */
extern GPTIMER_Config gGpTimerConfig[GPTIMER_NUM_INSTANCES];
/* Pinmux configuration array for PWM pins */
Pinmux_PerCfg_t pMux_PWM[2];

/**
 * @brief Configure and start a PWM channel using GPTIMER
 * @param pwmChannel PWM channel index (e.g., PWM0, PWM1, PWM2)
 * @param pwmFreq Desired PWM frequency in Hz
 * @param pwmDutyCycle Desired duty cycle percentage (0-100)
 * @return int32_t Returns 0 on success, negative value on failure
 */
int32_t PWM_ENABLE(uint8_t pwmChannel, uint32_t pwmFreq, uint32_t pwmDutyCycle)
{
    int32_t pwmDriverStatus = 0;

    /* ----------------------------------------------------------------------
     * Calculate timer counts for frequency and duty cycle
     * -------------------------------------------------------------------- */
    uint32_t pwmFreqCnt = (PWM0_TIMER_INPUT_CLK_HZ / pwmFreq);   /* Period count */
    uint32_t pwmDutyCnt = (pwmFreqCnt) - ((pwmDutyCycle * pwmFreqCnt) / 100);     /* Duty count */

    /* ----------------------------------------------------------------------
     * Configure pin multiplexing for PWM output
     * -------------------------------------------------------------------- */

    /* Select correct pin offset based on PWM channel */
    switch (pwmChannel) 
    {
        case PWM0: 
            pMux_PWM[0].offset = pPWM_Offset[PWM0]; 
            pMux_PWM[0].settings = (PIN_MODE(2) | PIN_PULL_DISABLE);     /* Set pin mode */
            break;
        case PWM1: 
            pMux_PWM[0].offset = pPWM_Offset[PWM1]; 
            pMux_PWM[0].settings = (PIN_MODE(2) | PIN_PULL_DISABLE);     /* Set pin mode */
            break;
        case PWM2: 
            pMux_PWM[0].offset = pPWM_Offset[PWM2]; 
            pMux_PWM[0].settings = (PIN_MODE(6) | PIN_PULL_DISABLE);     /* Set pin mode */
            break;
        default:   return -1;  /* Invalid channel */
    }
    
    pMux_PWM[1].settings = PINMUX_END;
    pMux_PWM[1].offset = PINMUX_END;
    
    /* Configure pin multiplexing for PWM output */
    Pinmux_config(pMux_PWM, PINMUX_DOMAIN_ID_MAIN);
    
    /* ----------------------------------------------------------------------
     * Close existing timer handle if already open
     * -------------------------------------------------------------------- */
    if (gGpTimerHandle[pwmChannel] != NULL) 
    {
        GPTIMER_close(gGpTimerHandle[pwmChannel]);
        gGpTimerHandle[pwmChannel] = NULL;
    }

    /* ----------------------------------------------------------------------
     * Configure GPTIMER parameters for PWM
     * -------------------------------------------------------------------- */
    gGpTimerParams[pwmChannel].counterInitVal = (PWM_TIMER_OVERFLOW - pwmFreqCnt);   /* Initial counter value */
    gGpTimerParams[pwmChannel].cntReloadVal   = (PWM_TIMER_OVERFLOW - pwmFreqCnt);   /* Reload value */
    gGpTimerConfig[pwmChannel].object->pwmConfig.cntCompareValPWM = (PWM_TIMER_OVERFLOW - pwmDutyCnt); /* Compare value for duty cycle */
    
    /* ----------------------------------------------------------------------
     * Open GPTIMER driver
     * -------------------------------------------------------------------- */
    gGpTimerHandle[pwmChannel] = GPTIMER_open(pwmChannel, &gGpTimerParams[pwmChannel]);
    if (gGpTimerHandle[pwmChannel] == NULL) 
    {
        DebugP_logError("GPTIMER open failed for instance %d !!!\r\n", pwmChannel);
        pwmDriverStatus = SystemP_FAILURE;
        return pwmDriverStatus;
    }

    /* ----------------------------------------------------------------------
     * Start the timer to generate PWM
     * -------------------------------------------------------------------- */
    GPTIMER_start(gGpTimerHandle[pwmChannel]);

    return pwmDriverStatus;  /* Success */
}

/**
 * @brief Disable a PWM channel and clean up resources
 * @param _pwmChannel PWM channel index to disable
 * @return int32_t Returns 0 on success, -1 on failure
 */
int32_t PWM_DISABLE(int8_t _pwmChannel)
{
    /* Validate PWM channel index */
    if (_pwmChannel > MAX_PWM_PINS) 
    {
        return -1;
    }
    
    /* Stop the PWM timer */
    GPTIMER_stop(gGpTimerHandle[_pwmChannel]);
    
    /* ----------------------------------------------------------------------
     * Close existing timer handle
     * -------------------------------------------------------------------- */
    if (gGpTimerHandle[_pwmChannel] != NULL) 
    {
        GPTIMER_close(gGpTimerHandle[_pwmChannel]);
        gGpTimerHandle[_pwmChannel] = NULL;
    }
    return 0;
}
