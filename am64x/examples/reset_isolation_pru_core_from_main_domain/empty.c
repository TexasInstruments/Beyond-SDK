/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <kernel/dpl/AddrTranslateP.h>


const unsigned int PRUFirmware[]= {
                                   0x240000e3,
                                   0x240002fe,
                                   0x240000fe,
                                   0xf1002382,
                                   0x6901e202,
                                   0xe1042382,
                                   0x7f0000fa,
                                   0x2a000000};




PRUICSS_Handle gPruIcss0Handle;
#define PRUICSS_PRUx                PRUICSS_PRU0
#define PRU_ISO_ADDRESS_OFFSET 0x3002601CUL

#define LPSC_OFFSET 0xA00U
#define LPSC_MODULE_NO  30U

/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */
void pru_init(void);








extern uint32_t Board_getGpioButtonIntrNum(void);
extern uint32_t Board_getGpioButtonSwitchNum(void);
static void GPIO_bankIsrFxn(void *args);
void gpio_input_interrupt_main(void *args);
extern void Board_gpioInit(void);

uint32_t            gGpioBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;
HwiP_Object         gGpioHwiObject;
volatile uint32_t   gGpioIntrDone = 0;




void empty_main(void *args)
{


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    gpio_input_interrupt_main(NULL);


    uint32_t Time = 0UL;

    uint32_t BaseAddr;
    volatile uint32_t *addr;
    uint32_t iso_req_status = 0UL;



    BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( PRU_ISO_ADDRESS_OFFSET );
    addr = (uint32_t *)BaseAddr;


     /*Enable ICSSG0 Reset Isolation by setting RESETISO and BLKCHIP1RST bits in PSC0_MDCTL30 register. */
      (*((volatile uint32_t *)(CSL_PSC0_BASE + LPSC_OFFSET + (LPSC_MODULE_NO * 4 ) ))) |= ( (1<<11) | (1<<12) );



     BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + 4U);
     iso_req_status = CSL_REG32_RD(BaseAddr);

     /* Reading PRU Memory address */
    if(iso_req_status == 0x01)
    {
         DebugP_log("PRU_ICSSG0 is Isolated from Main Domain !!\r\n ");
         *addr |= (0x1U);



         BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE);
         iso_req_status = CSL_REG32_RD(BaseAddr);
         DebugP_log("PRU Request value  at 0x30000000UL !!:- %d\r\n", iso_req_status);
         CSL_REG32_WR(BaseAddr,0x00);


         BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + 4U);
         iso_req_status = CSL_REG32_RD(BaseAddr);
         DebugP_log("PRU Response value  at 0x30000004UL !!:- %d\r\n", iso_req_status);
         CSL_REG32_WR(BaseAddr,0x00);


    }
    else
    {
        DebugP_log("PRU_ICSSG0 is not Isolated from Main Domain !!\r\n ");

        /*PRU init should be done for only one Time*/
        gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

        /*iso_req_status = PRUICSS_intcInit(gPruIcss0Handle, &icss0_intc_initdata);
        DebugP_assert(SystemP_SUCCESS == iso_req_status);*/


        pru_init();

    }

    while(1)
    {

            Time = Time + 1UL;

            ClockP_sleep(1);


            DebugP_log("Timer in sec!!:- %d\r\n", Time);

            BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE);
            iso_req_status = CSL_REG32_RD(BaseAddr);
            DebugP_log("PRU Request value  at 0x30000000UL !!:- %d\r\n", iso_req_status);


            BaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + 4U);
            iso_req_status = CSL_REG32_RD(BaseAddr);
            DebugP_log("PRU Response value  at 0x30000004UL !!:- %d\r\n", iso_req_status);

    }



    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}



void pru_init()
{
    int status;


    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));
    DebugP_assert(status != 0);
    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx));
    DebugP_assert(status != 0);
    status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);


    status = PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx), 0,
                       (uint32_t *) PRUFirmware, sizeof(PRUFirmware));
    DebugP_assert(status != 0);

    status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);


}


void gpio_input_interrupt_main(void *args)
{
    int32_t         retVal;
    uint32_t        pinNum, intrNum;
    HwiP_Params     hwiPrms;
    uint32_t        bankNum;


    Board_gpioInit();


    DebugP_log("GPIO Input Interrupt Test Started ...\r\n");
    DebugP_log("GPIO Interrupt Configured for Rising Edge (Button release will trigger interrupt) ...\r\n");

    pinNum          = GPIO_PUSH_BUTTON_PIN;
    intrNum         = Board_getGpioButtonIntrNum();
    bankNum         = GPIO_GET_BANK_INDEX(pinNum);

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);

    /* Setup GPIO for interrupt generation */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_DIR);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    GPIO_bankIntrEnable(gGpioBaseAddr, bankNum);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &GPIO_bankIsrFxn;
    hwiPrms.args     = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );




}

static void GPIO_bankIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args;
    uint32_t    bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t    intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);




    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    /* Per pin interrupt handling */
    if(intrStatus & pinMask)
    {

           gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE);

            /* Send ISO_Request*/
            /* Writing data in to PRU Memory address */
            CSL_REG32_WR(gGpioBaseAddr,0x01);

            DebugP_log("Warm Reset is Triggered !!\r\n");
            Sciclient_pmDeviceReset(SystemP_WAIT_FOREVER);

    }
    else
    {

    }

}
