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
#include <drivers/sciclient.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient/include/tisci/security/tisci_soc_uid.h>

void read_soc_uid_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    struct tisci_msg_get_soc_uid_req req;

    Sciclient_ReqPrm_t sci_req;
    sci_req.messageType    = TISCI_MSG_GET_SOC_UID;
    sci_req.flags          = TISCI_MSG_FLAG_AOP;
    sci_req.pReqPayload    = (const uint8_t*)&req;
    sci_req.reqPayloadSize = (uint32_t)sizeof(req);
    sci_req.timeout        = SystemP_WAIT_FOREVER;

    struct tisci_msg_get_soc_uid_resp resp;

    Sciclient_RespPrm_t sci_resp;
    sci_resp.flags           = 0;
    sci_resp.pRespPayload    = (uint8_t*)&resp;
    sci_resp.respPayloadSize = (uint32_t)sizeof(resp);

    int32_t status = Sciclient_service(&sci_req, &sci_resp);

    DebugP_assert(status == SystemP_SUCCESS);
    DebugP_assert((sci_resp.flags & TISCI_MSG_FLAG_ACK) == TISCI_MSG_FLAG_ACK);

    DebugP_log("SOC UID: ");
    for(int32_t i = 0; i < UID_LEN_WORDS; i++)
    {
        DebugP_log("%x", resp.soc_uid[i]);
    }

    DebugP_log("\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
