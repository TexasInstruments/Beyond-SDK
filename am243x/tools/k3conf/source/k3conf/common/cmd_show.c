/*
 * K3CONF Command Show
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - https://www.ti.com/
 * Tushar Thakur <tushar@ti.com>
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
#include "tisci.h"
#include "socinfo.h"
#include "help.h"
#include "k3conf.h"
#include <kernel/dpl/DebugP.h>
#include "../soc/am243x/am243x_host_info.h"

int show_hosts_info(void)
{
	struct ti_sci_host_info *hosts = soc_info.sci_info.host_info;
	char temp[116];
	memset(temp, '=', 111);
	strncpy(temp+111, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-15s| %-18s| %-60s|\r\n", "Host ID","Host Name", "Security Status", "Description");
	DebugP_log("%s", temp);
	for(uint32_t itr = 0; itr < soc_info.sci_info.num_hosts; itr++) 
	{
		DebugP_log("|%-10u| %-15s| %-18s| %-60s|\r\n", hosts[itr].host_id, hosts[itr].host_name, hosts[itr].security_status, hosts[itr].description);
	}
	DebugP_log("%s", temp);
	return 0;	
}

static int show_clocks_info(int argc, char *argv[])
{
	struct ti_sci_clocks_info *clocks = soc_info.sci_info.clocks_info;
	uint32_t dev_id = INVALID_NUM;
	if(argc >= 1) {
		int ret = sscanf(argv[0], "%u", &dev_id);
		if(ret != 1)
		{
			return -1;
		}
		if(!isValidDeviceId(dev_id))
		{
			return -1;
		}	
	}
	char temp[233];
	memset(temp, '=', 228);
	strncpy(temp+228, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-10s| %-100s| %-100s|\r\n", "Dev ID","Clock ID", "Clock Name", "Clock Function");
	DebugP_log("%s", temp);

	for(uint32_t itr = 0; itr < soc_info.sci_info.num_clocks; itr++) 
	{
		if(argc == 0 || dev_id == clocks[itr].dev_id)
		{
			DebugP_log("|%-10u| %-10u| %-100s| %-100s|\r\n", clocks[itr].dev_id, clocks[itr].clk_id, clocks[itr].clk_name, clocks[itr].clk_function);
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

static int show_devices_info(int argc, char *argv[])
{
	struct ti_sci_devices_info *devices = soc_info.sci_info.devices_info;
	uint32_t dev_id = INVALID_NUM;
	if(argc >= 1)
	{
		int ret = sscanf(argv[0], "%u", &dev_id);
		if(ret != 1)
		{
			return -1;
		}
		if(!isValidDeviceId(dev_id))
		{
			return -1;
		}
	}
	char temp[69];
	memset(temp, '=', 64);
	strncpy(temp+64, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-50s|\r\n", "Dev ID","Device Name");
	DebugP_log("%s", temp);
	for(uint32_t itr = 0; itr < soc_info.sci_info.num_devices; itr++) 
	{
		if(argc == 0 || dev_id == devices[itr].dev_id)
		{
			DebugP_log("|%-10u| %-50s|\r\n", devices[itr].dev_id, devices[itr].name);
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

static int show_rm_info(int argc, char *argv[])
{
	struct ti_sci_rm_info *rm = soc_info.sci_info.rm_info;
	uint32_t dev_id = INVALID_NUM;
	if(argc >= 1) 
	{
		int ret = sscanf(argv[0], "%u", &dev_id);
		if(ret != 1)
		{
			return -1;
		}
		if(!isValidDeviceId(dev_id))
		{
			return -1;
		}
		
		uint8_t isFound = 0;
		for(uint32_t itr = 0; itr < soc_info.sci_info.num_res; itr++) 
		{
			if(rm[itr].dev_id == dev_id)
			{
				isFound = 1;
				break;
			}
		}
		if(!isFound)
		{
			DebugP_log("Resources for type %d are not managed by SYSFW\r\n", dev_id);
			return -1;
		}
	}
	char temp[249];
	memset(temp, '=', 244);
	strncpy(temp+244, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-12s| %-10s| %-100s| %-12s| %-100s|\r\n", "Unique type","Dev ID", "Device Name", "Subtype ID", "Subtype Name");
	DebugP_log("%s", temp);
	for(uint32_t itr = 0; itr < soc_info.sci_info.num_res; itr++) 
	{
		if(argc == 0 || dev_id == rm[itr].dev_id)
		{
			DebugP_log("|0x%-10x| %-10u| %-100s| %-12u| %-100s|\r\n", rm[itr].utype, rm[itr].dev_id, rm[itr].device_name, rm[itr].subtype_id, rm[itr].subtype_name);
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

static int show_processors_info(void)
{
	struct ti_sci_processors_info *processors = soc_info.sci_info.processors_info;
	char temp[74];
	memset(temp, '=', 69);
	strncpy(temp+69, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-15s| %-50s|\r\n", "Processor ID", "Processor Name");
	DebugP_log("%s", temp);
	for(uint32_t itr = 0; itr < soc_info.sci_info.num_processors; itr++) 
	{
		DebugP_log("|%-15u| %-50s|\r\n", processors[itr].processor_id, processors[itr].name);
	}
	DebugP_log("%s", temp);
	return 0;
}

int process_show_command(int argc, char *argv[])
{
	int ret;

	if (argc < 1) {
		help(HELP_SHOW);
		return -1;
	}

	if (!strncmp(argv[0], "host", 4)) {
		ret = show_hosts_info();
		if (ret)
			help(HELP_SHOW_HOST);
	} else if (!strncmp(argv[0], "device", 6)) {
		argc--;
		argv++;
		ret = show_devices_info(argc, argv);
		if (ret) {
			DebugP_log("Invalid device arguments\r\n");
			help(HELP_SHOW_DEVICE);
		}
	} else if (!strncmp(argv[0], "clock", 5)) {
		argc--;
		argv++;
		ret = show_clocks_info(argc, argv);
		if (ret) {
			DebugP_log("Invalid clock arguments\r\n");
			help(HELP_SHOW_CLOCK);
		}
	} else if(!strncmp(argv[0], "processor", 9)) {
		ret = show_processors_info();
		if (ret)
			help(HELP_SHOW_PROCESSOR);
	} else if(!strncmp(argv[0], "rm", 2)) {
		argc--;
		argv++;
		ret = show_rm_info(argc, argv);
		if (ret) {
			DebugP_log("Invalid device_id arguments\r\n");
			help(HELP_SHOW_RM);
		}
	} else if (!strcmp(argv[0], "--help")) {
		help(HELP_SHOW);
		return 0;
	} else {
		DebugP_log("Invalid argument %s\r\n", argv[0]);
		help(HELP_SHOW);
		return -1;
	}
	return ret;
}
