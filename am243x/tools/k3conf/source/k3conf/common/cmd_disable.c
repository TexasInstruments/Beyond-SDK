/*
 * K3CONF Command Disable
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <drivers/soc/am64x_am243x/soc.h>
#include "tisci.h"
#include "socinfo.h"
#include "help.h"
#include "k3conf.h"

static int disable_device(int argc, char *argv[])
{
	uint32_t dev_id, ret;

	if (argc < 1)
		return -1;

	ret = sscanf(argv[0], "%u", &dev_id);
	if (ret != 1)
		return -1;

	ret = Sciclient_pmSetModuleState(dev_id, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
	(TISCI_MSG_FLAG_AOP | TISCI_MSG_FLAG_DEVICE_RESET_ISO), SystemP_WAIT_FOREVER);
	
	if (ret)
		return ret;

	return dump_devices_info(argc, argv);
}

static int disable_clock(int argc, char *argv[])
{
	uint32_t dev_id, clk_id, ret;

	if (argc < 2)
		return -1;

	ret = sscanf(argv[0], "%u", &dev_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[1], "%u", &clk_id);
	if (ret != 1)
		return -1;

	ret = Sciclient_pmModuleClkRequest(dev_id, clk_id, TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ, 0U, SystemP_WAIT_FOREVER);
	if (ret)
		return ret;

	return dump_clocks_info(argc, argv);
}

int process_disable_command(int argc, char *argv[])
{
	int ret;

	if (argc < 1) {
		help(HELP_DISABLE);
		return -1;
	}

	if (!strncmp(argv[0], "device", 6)) {
		argc--;
		argv++;
		ret = disable_device(argc, argv);
		if (ret) {
			DebugP_log("Invalid device arguments\r\n");
			help(HELP_DISABLE_DEVICE);
		}
	} else if (!strncmp(argv[0], "clock", 5)) {
		argc--;
		argv++;
		ret = disable_clock(argc, argv);
		if (ret) {
			DebugP_log("Invalid clock arguments\r\n");
			help(HELP_DISABLE_CLOCK);
		}
	} else if (!strcmp(argv[0], "--help")) {
		help(HELP_DISABLE);
		return 0;
	} else {
		DebugP_log("Invalid argument %s\r\n", argv[1]);
		help(HELP_DISABLE);
		return -1;
	}
	return ret;
}
