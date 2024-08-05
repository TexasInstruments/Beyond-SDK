/*
 * K3 SoC detection and helper apis
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - https://www.ti.com/
 *	Tushar Thakur <tushar@ti.com>
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

#include "socinfo.h"
#include <string.h>
#include <drivers/sciclient.h>
#include <drivers/hw_include/hw_types.h>
#include "../soc/am243x/am243x_host_info.h"
#include "../soc/am243x/am243x_processors_info.h"
#include "../soc/am243x/am243x_devices_info.h"
#include "../soc/am243x/am243x_clocks_info.h"
#include "../soc/am243x/am243x_rm_info.h"



struct k3conf_soc_info soc_info;
struct tisci_msg_version_req grequest;
struct tisci_msg_version_resp gresponse;

void k3conf_get_version(struct tisci_msg_version_req *request, struct tisci_msg_version_resp *response);

static void am243x_init(void)
{
	struct ti_sci_info *sci_info = &soc_info.sci_info;

	sci_info->host_info = am243x_host_info;
	sci_info->num_hosts = AM243X_MAX_HOST_IDS;
	sci_info->processors_info = am243x_processors_info;
	sci_info->num_processors = AM243X_MAX_PROCESSORS_IDS;
	sci_info->devices_info = am243x_devices_info;
	sci_info->num_devices = AM243X_MAX_DEVICES;
	sci_info->clocks_info = am243x_clocks_info;
	sci_info->num_clocks = AM243X_MAX_CLOCKS;
	sci_info->rm_info = am243x_rm_info;
	sci_info->num_res = AM243X_MAX_RES;
	
	sci_info->version.abi_major = gresponse.abi_major;
	sci_info->version.abi_minor = gresponse.abi_minor;
	sci_info->version.firmware_version = gresponse.version;
	strcpy(sci_info->version.firmware_description, gresponse.str);
}

int soc_init()
{
	memset(&soc_info, 0, sizeof(soc_info));

	k3conf_get_version(&grequest, &gresponse);
	soc_info.soc_name = "AM243x";
	am243x_init();

	return 0;
}
