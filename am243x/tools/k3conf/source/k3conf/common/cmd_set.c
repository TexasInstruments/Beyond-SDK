/*
 * K3CONF Command Set
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
#include <drivers/bootloader/soc/am64x_am243x/bootloader_soc.h>
#include "tisci.h"
#include "socinfo.h"
#include "help.h"
#include "k3conf.h"

#define HELP_SET_PARENT_URL1 "http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/pm/clocks.html#power-management-clock-frequency-configuration-example-with-mux-programming"
#define HELP_SET_PARENT_URL2 "http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/pm/clocks.html#tisci-msg-set-clock-parent"

static int set_clock(int argc, char *argv[])
{
	uint32_t dev_id, clk_id, ret;
	uint64_t freq;

	if (argc < 3)
		return -1;

	ret = sscanf(argv[0], "%u", &dev_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[1], "%u", &clk_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[2], "%llu", &freq);
	if (ret != 1)
		return -1;

	ret =  SOC_moduleSetClockFrequency(dev_id, clk_id, freq);
	if (ret)
		return ret;

	return dump_clocks_info(argc, argv);
}

static int set_clock_parent(int argc, char *argv[])
{
	uint32_t dev_id, clk_id, parent_clk_id;
	int ret;

	if (argc < 3)
		return -1;

	ret = sscanf(argv[0], "%u", &dev_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[1], "%u", &clk_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[2], "%u", &parent_clk_id);
	if (ret != 1)
		return -1;

	ret = Sciclient_pmSetModuleClkParent(dev_id, clk_id, parent_clk_id,SystemP_WAIT_FOREVER);
	if (ret) {
		DebugP_log("Request to set parent failed: %d\r\n",ret);
		DebugP_log("Clock state is probably wrong!\r\n");
		uint32_t clk_sts = 0;
		Sciclient_pmModuleGetClkStatus(dev_id, clk_id, &clk_sts, SystemP_WAIT_FOREVER);
		DebugP_log("Clock state of clk_id %d: %u\r\n",
			clk_id, clk_sts);
		Sciclient_pmModuleGetClkStatus(dev_id, parent_clk_id, &clk_sts, SystemP_WAIT_FOREVER);
		DebugP_log("Clock state of parent_clk_id %d: %u\r\n",
			parent_clk_id, clk_sts);
		DebugP_log("\r\nRefer to:\r\n\t%s\r\n\t%s\r\n",
			HELP_SET_PARENT_URL1, HELP_SET_PARENT_URL2);

		return ret;
	}

	return dump_clock_parent_info(argc - 1, argv);
}

static int set_processor(int argc, char *argv[])
{
	uint32_t processor_id, ret;
	uint64_t freq;

	if (argc < 2)
		return -1;

	ret = sscanf(argv[0], "%u", &processor_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[1], "%llu", &freq);
	if (ret != 1)
		return -1;

	ret =  Bootloader_socCpuSetClock(processor_id, freq);
	if (ret)
		return ret;

	return dump_processors_info(argc, argv);
}

int process_set_command(int argc, char *argv[])
{
	int ret;

	if (argc < 1) {
		help(HELP_SET);
		return -1;
	}

	if (!strncmp(argv[0], "clock", 5)) {
		argc--;
		argv++;
		ret = set_clock(argc, argv);
		if (ret) {
			DebugP_log("Invalid clock arguments\r\n");
			help(HELP_SET_CLOCK);
		}
	} else if (!strncmp(argv[0], "parent_clock", 5)) {
		argc--;
		argv++;
		ret = set_clock_parent(argc, argv);
		if (ret) {
			if (ret == -1) {
				DebugP_log("Invalid parent_clock arguments\r\n");
				help(HELP_SET_CLOCK_PARENT);
			}
		}
	} else if (!strncmp(argv[0], "processor", 5)) {
		argc--;
		argv++;
		ret = set_processor(argc, argv);
		if (ret) {
			if (ret == -1) {
				DebugP_log("Invalid processor arguments\r\n");
				help(HELP_SET_PROCESSOR);
			}
		}
	} else if (!strcmp(argv[0], "--help")) {
		help(HELP_SET);
		return 0;
	} else {
		DebugP_log("Invalid argument %s\r\n", argv[1]);
		help(HELP_SET);
		return -1;
	}
	return ret;
}
