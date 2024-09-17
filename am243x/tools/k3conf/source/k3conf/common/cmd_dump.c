/*
 * K3CONF Command Dump
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - https://www.ti.com/
 *  Tushar Thakur <tushar@ti.com>
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
#include <drivers/bootloader.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/soc/am64x_am243x/soc.h>
#include <drivers/sciclient.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_hosts.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_boardcfg_constraints.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_devices.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_resasg_types.h>

struct tisci_local_rm_boardcfg
{
	struct tisci_boardcfg_rm rm_boardcfg;
	/**< Board configuration parameter */
	struct tisci_boardcfg_rm_resasg_entry resasg_entries[TISCI_RESASG_ENTRIES_MAX];
	/**< Resource assignment entries */
};

extern struct tisci_local_rm_boardcfg gBoardConfigLow_rm;


static void get_host_name(uint32_t host_id, char *host_name)
{
	switch (host_id)
	{
	case TISCI_HOST_ID_DMSC:
		strcpy(host_name, "DMSC");
		break;
	case TISCI_HOST_ID_MAIN_0_R5_0:
		strcpy(host_name, "MAIN_0_R5_0");
		break;
	case TISCI_HOST_ID_MAIN_0_R5_1:
		strcpy(host_name, "MAIN_0_R5_1");
		break;
	case TISCI_HOST_ID_MAIN_0_R5_2:
		strcpy(host_name, "MAIN_0_R5_2");
		break;
	case TISCI_HOST_ID_MAIN_0_R5_3:
		strcpy(host_name, "MAIN_0_R5_3");
		break;
	case TISCI_HOST_ID_M4_0:
		strcpy(host_name, "M4_0");
		break;
	case TISCI_HOST_ID_MAIN_1_R5_0:
		strcpy(host_name, "MAIN_1_R5_0");
		break;
	case TISCI_HOST_ID_MAIN_1_R5_1:
		strcpy(host_name, "MAIN_1_R5_1");
		break;
	case TISCI_HOST_ID_MAIN_1_R5_2:
		strcpy(host_name, "MAIN_1_R5_2");
		break;
	case TISCI_HOST_ID_MAIN_1_R5_3:
		strcpy(host_name, "MAIN_1_R5_3");
		break;
	case TISCI_HOST_ID_A53_0:
		strcpy(host_name, "A53_0");
		break;
	case TISCI_HOST_ID_A53_1:
		strcpy(host_name, "A53_1");
		break;
	case TISCI_HOST_ID_A53_2:
		strcpy(host_name, "A53_2");
		break;
	case TISCI_HOST_ID_A53_3:
		strcpy(host_name, "A53_3");
		break;
	case TISCI_HOST_ID_A53_4:
		strcpy(host_name, "A53_4");
		break;
	case TISCI_HOST_ID_ICSSG_0:
		strcpy(host_name, "ICSSG_0");
		break;
	case TISCI_HOST_ID_ICSSG_1:
		strcpy(host_name, "ICSSG_1");
		break;
	default:
		strcpy(host_name, "ALL");
	}
}

int dump_clocks_info(int argc, char *argv[])
{
	struct ti_sci_clocks_info *clocks = soc_info.sci_info.clocks_info;
	uint32_t dev_id = INVALID_NUM;
	if (argc >= 1)
	{
		int ret = sscanf(argv[0], "%u", &dev_id);
		if (ret != 1)
		{
			return -1;
		}
		if(!isValidDeviceId(dev_id))
		{
			return -1;
		}
	}
	char temp[160];
	memset(temp, '=', 148);
	strncpy(temp + 148, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-10s| %-100s| %-20s|\r\n", "Dev ID", "Clock ID", "Clock Name", "Frequency");
	DebugP_log("%s", temp);
	uint64_t clk_rate = 0;
	for (int itr = 0; itr < soc_info.sci_info.num_clocks; itr++)
	{
		if (argc == 0 || clocks[itr].dev_id == dev_id)
		{
			clk_rate = 0;
			SOC_moduleGetClockFrequency(clocks[itr].dev_id, clocks[itr].clk_id, &clk_rate);
			DebugP_log("|%-10u| %-10u| %-100s| %-20llu|\r\n", clocks[itr].dev_id, clocks[itr].clk_id, clocks[itr].clk_name, clk_rate);
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

int dump_clock_parent_info(int argc, char *argv[])
{
	struct ti_sci_clocks_info *clocks = soc_info.sci_info.clocks_info;

	if (argc < 2)
		return -1;

	uint32_t dev_id = 0xFFFFFFFF, clk_id = 0xFFFFFFFF;

	int ret = sscanf(argv[0], "%u", &dev_id);
	if (ret != 1)
		return -1;

	ret = sscanf(argv[1], "%u", &clk_id);
	if (ret != 1)
		return -1;

	uint8_t found = 0;
	uint32_t itr = 0;
	for(itr=0; itr<soc_info.sci_info.num_clocks; itr++)
	{
		if(dev_id == clocks[itr].dev_id && clk_id == clocks[itr].clk_id)
		{
			found = 1;
			break;
		}
	}
	if(! found)
	{
		return -1;
	}

	char temp[180];
	memset(temp, '=', 170);
	strncpy(temp + 170, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-168s|\r\n", "Clock Information");
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-10s| %-100s| %-20s| %-20s|\r\n", "Dev ID", "Clock ID", "Clock Name", "Status", "Frequency");
	DebugP_log("%s", temp);
	uint64_t clk_rate = 0;
	uint32_t clk_status = 0;
	SOC_moduleGetClockFrequency(dev_id, clk_id, &clk_rate);
	Sciclient_pmModuleGetClkStatus(dev_id, clk_id, &clk_status, SystemP_WAIT_FOREVER);
	char *clk_state = (clk_status) ? "CLK_STATE_READY" : "CLK_STATE_UNREADY";
	DebugP_log("|%-10u| %-10u| %-100s| %-20s| %-20llu|\r\n", dev_id, clk_id, clocks[itr].clk_name, clk_state, clk_rate);
	DebugP_log("%s", temp);


	uint32_t numParent = 0;
	Sciclient_pmGetModuleClkNumParent(dev_id, clk_id, &numParent, SystemP_WAIT_FOREVER);
	if(numParent == 0)
	{
		DebugP_log("No Parent Clock information available for selected clock\r\n");
		return 0;
	}

	DebugP_log("\r\n");
	memset(temp, '=', 170);
	strncpy(temp + 170, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-168s|\r\n", "Clock Parent Information");
	DebugP_log("%s", temp);
	DebugP_log("|%-10s| %-10s| %-100s| %-20s| %-20s|\r\n", "Selected", "Clock ID", "Clock Name", "Status", "Frequency");
	DebugP_log("%s", temp);
	for(uint32_t idx=itr+1; idx<itr+numParent; idx++)
	{
		clk_rate = 0, clk_status = 0;
		SOC_moduleGetClockFrequency(dev_id, clk_id, &clk_rate);
		Sciclient_pmModuleGetClkStatus(dev_id, clk_id, &clk_status, SystemP_WAIT_FOREVER);
		clk_state = (clk_status) ? "CLK_STATE_READY" : "CLK_STATE_UNREADY";
		DebugP_log("|%-10s| %-10u| %-100s| %-20s| %-20llu|\r\n", "  ==> ", clocks[idx].clk_id, clocks[idx].clk_name, clk_state, clk_rate);
	}
	DebugP_log("%s", temp);

	return 0;
}

int dump_devices_info(int argc, char *argv[])
{
	struct ti_sci_devices_info *devices = soc_info.sci_info.devices_info;
	uint32_t dev_id = INVALID_NUM;
	if (argc >= 1)
	{
		int ret = sscanf(argv[0], "%u", &dev_id);
		if (ret != 1)
		{
			return -1;
		}
		if(!isValidDeviceId(dev_id))
		{
			return -1;
		}
	}
	char temp[105];
	memset(temp, '=', 96);
	strncpy(temp + 96, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-15s| %-50s| %-25s|\r\n", "Device ID", "Device Name", "Device Status");
	DebugP_log("%s", temp);
	uint32_t moduleState = 0, resetState = 0, contextLossState = 0;
	for (int itr = 0; itr < soc_info.sci_info.num_devices; itr++)
	{
		if (argc == 0 || devices[itr].dev_id == dev_id)
		{
			moduleState = 0, resetState = 0, contextLossState = 0;
			Sciclient_pmGetModuleState(devices[itr].dev_id, &moduleState, &resetState, &contextLossState, SystemP_WAIT_FOREVER);
			char *device_state = (moduleState) ? "DEVICE_STATE_ON" : "DEVICE_STATE_OFF";
			DebugP_log("|%-15u| %-50s| %-25s|\r\n", devices[itr].dev_id, devices[itr].name, device_state);
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

int dump_processors_info(int argc, char *argv[])
{
	struct ti_sci_processors_info *processors = soc_info.sci_info.processors_info;
	uint32_t processor_id = INVALID_NUM;
	if (argc >= 1)
	{
		int ret = sscanf(argv[0], "%u", &processor_id);
		if (ret != 1)
		{
			return -1;
		}
		if(!isValidProcessorId(processor_id))
		{
			return -1;
		}
	}
	char temp[75];
	memset(temp, '=', 69);
	strncpy(temp + 69, "\r\n", 5);
	DebugP_log("%s", temp);
	DebugP_log("|%-13s| %-8s| %-20s| %-20s|\r\n", "Processor ID", "Dev ID", "CPU Name", "Frequency");
	DebugP_log("%s", temp);
	for (int itr = 0; itr < soc_info.sci_info.num_processors; itr++)
	{
		if (argc == 0 || processors[itr].processor_id == processor_id)
		{
			DebugP_log("|%-13d| %-8d| %-20s| %-20llu|\r\n", processors[itr].processor_id, processors[itr].dev_id, processors[itr].name, Bootloader_socCpuGetClock(processors[itr].processor_id));
		}
	}
	DebugP_log("%s", temp);
	return 0;
}

uint32_t dump_rm_host_resource(struct tisci_boardcfg_rm_host_cfg_entry *host_cfg, uint32_t host_id)
{
	struct ti_sci_rm_info *rm = soc_info.sci_info.rm_info;
	uint32_t size = sizeof(gBoardConfigLow_rm.rm_boardcfg.host_cfg.host_cfg_entries) / sizeof(host_cfg[0]);
	uint32_t num_host = 0;
	char host_name[50] = {0};
	uint8_t isFound = 0;
	for (num_host = 0; num_host < size; num_host++)
	{
		if (host_id == host_cfg[num_host].host_id && host_cfg[num_host].allowed_priority != 0)
		{
			get_host_name(host_cfg[num_host].host_id, host_name);
			isFound = 1;
			break;
		}
	}

	if(!isFound)
	{
		DebugP_log("No resource information for host with host_id = %u\r\n", host_id);
		return 0;
	}

	char temp[37 + 44];
	memset(temp, '=', 37 + 38);
	strncpy(temp + (37 + 38), "\r\n", 5);

	DebugP_log("%s", temp);
	DebugP_log("|%-12s| %-8s| %-12s| ", "Unique type", "Dev ID", "Subtype ID");
	DebugP_log("%-35s| ", host_name);
	DebugP_log("\r\n");
	DebugP_log("%s", temp);

	struct tisci_msg_rm_get_resource_range_req req = {{0}};
	struct tisci_msg_rm_get_resource_range_resp res = {{0}};
	req.hdr.type = TISCI_MSG_RM_GET_RESOURCE_RANGE;
	req.hdr.flags = TISCI_MSG_FLAG_AOP;
	req.hdr.host = host_id;
	req.secondary_host = host_id;

	for (int itr = 0; itr < soc_info.sci_info.num_res; itr++)
	{
		req.type = rm[itr].dev_id;
		req.subtype = rm[itr].subtype_id;
		DebugP_log("|0x%-10x| %-8u| %-12u| ", rm[itr].utype, rm[itr].dev_id, rm[itr].subtype_id);
		
		Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
		char str[2][15] = {{0}, {0}};
		if (res.range_num > 0)
		{
			snprintf(str[0], 15, "[%4hu + %-4hu]", res.range_start, res.range_num);
		}
		if (res.range_num_sec > 0)
		{
			snprintf(str[1], 15, "(%4hu + %-4hu)", res.range_start_sec, res.range_num_sec);
		}
		DebugP_log("%-17s %-17s|\r\n", str[0], str[1]);
	}
	DebugP_log("%s", temp);
	return 0;
}


static int dump_rm_info(int argc, char *argv[])
{
	struct ti_sci_rm_info *rm = soc_info.sci_info.rm_info;

	struct tisci_boardcfg_rm_host_cfg_entry *host_cfg = gBoardConfigLow_rm.rm_boardcfg.host_cfg.host_cfg_entries;
	uint32_t size = sizeof(gBoardConfigLow_rm.rm_boardcfg.host_cfg.host_cfg_entries) / sizeof(host_cfg[0]);
	uint32_t dev_id = INVALID_NUM, sub_type = INVALID_NUM;
	uint32_t host_id = 0;
	
	if (argc >= 1)
	{
		if(strncmp("-h", argv[0], 2) != 0)
		{
			int ret = sscanf(argv[0], "%u", &dev_id);
			if (ret != 1)
			{
				return -1;
			}
			if(!isValidDeviceId(dev_id))
			{
				return -1;
			}
			if(argc == 2)
			{
				ret = sscanf(argv[1], "%u", &sub_type);
				if (ret != 1)
				{
					return -1;
				}
			}
		}
		else if(argc == 2)
		{
			int ret = sscanf(argv[1], "%u", &host_id);
			if (ret != 1)
			{
				return -1;
			}
			if(host_id != 0 && host_id != 128)
			{
				return dump_rm_host_resource(host_cfg, host_id);
			}
		}
		else
		{
			return -1;
		}
	}
	
	uint32_t num_host = 0;
	char host_name[32][50];
	for (num_host = 0; num_host < size; num_host++)
	{
		if (host_cfg[num_host].allowed_priority != 0)
		{
			char name[50];
			get_host_name(host_cfg[num_host].host_id, &name[0]);
			strcpy(host_name[num_host], name);
		}
		else
		{
			break;
		}
	}

	char temp[num_host * 37 + 44];
	memset(temp, '=', num_host * 37 + 38);
	strncpy(temp + (num_host * 37 + 38), "\r\n", 5);

	DebugP_log("%s", temp);
	DebugP_log("|%-12s| %-8s| %-12s| ", "Unique type", "Dev ID", "Subtype ID");
	for (int idx = 0; idx < num_host; idx++)
	{
		DebugP_log("%-35s| ", host_name[idx]);
	}
	DebugP_log("\r\n");
	DebugP_log("%s", temp);

	struct tisci_msg_rm_get_resource_range_req req = {{0}};
	struct tisci_msg_rm_get_resource_range_resp res = {{0}};
	req.hdr.type = TISCI_MSG_RM_GET_RESOURCE_RANGE;
	req.hdr.flags = TISCI_MSG_FLAG_AOP;
	uint8_t res_found = 0;
	for (int itr = 0; itr < soc_info.sci_info.num_res; itr++)
	{
		if(dev_id == INVALID_NUM || dev_id == rm[itr].dev_id)
		{
			req.type = rm[itr].dev_id;
			if(sub_type == INVALID_NUM || rm[itr].subtype_id == sub_type)
			{
				req.subtype = rm[itr].subtype_id;
				DebugP_log("|0x%-10x| %-8u| %-12u| ", rm[itr].utype, rm[itr].dev_id, rm[itr].subtype_id);
				for (int idx = 0; idx < num_host; idx++)
				{
					req.hdr.host = host_cfg[idx].host_id;
					req.secondary_host = host_cfg[idx].host_id;
					Sciclient_rmGetResourceRange(&req, &res, SystemP_WAIT_FOREVER);
					char str[2][15] = {{0}, {0}};
					if (res.range_num > 0)
					{
						snprintf(str[0], 15, "[%4hu + %-4hu]", res.range_start, res.range_num);
						res_found = 1;
					}
					if (res.range_num_sec > 0)
					{
						snprintf(str[1], 15, "(%4hu + %-4hu)", res.range_start_sec, res.range_num_sec);
						res_found = 1;
					}
					DebugP_log("%-17s %-17s| ", str[0], str[1]);
				}
				DebugP_log("\r\n");
			}
		}
	}
	if(!res_found)
	{
		DebugP_log("%s\r\n", "No Information for selected type or subtype");
	}
	DebugP_log("%s", temp);
	return 0;
}

int process_dump_command(int argc, char *argv[])
{
	int ret;

	if (argc < 1)
	{
		help(HELP_DUMP);
		return -1;
	}

	if (!strncmp(argv[0], "device", 6))
	{
		argc--;
		argv++;
		ret = dump_devices_info(argc, argv);
		if (ret)
		{
			DebugP_log("Invalid device arguments\r\n");
			help(HELP_DUMP_DEVICE);
		}
	}
	else if (!strncmp(argv[0], "clock", 5))
	{
		argc--;
		argv++;
		ret = dump_clocks_info(argc, argv);
		if (ret)
		{
			DebugP_log("Invalid clock arguments\r\n");
			help(HELP_DUMP_CLOCK);
		}
	} else if (!strncmp(argv[0], "parent_clock", 12)) {
		argc--;
		argv++;
		ret = dump_clock_parent_info(argc, argv);
		if (ret) {
			DebugP_log("Invalid clock_parent arguments\r\n");
			help(HELP_DUMP_CLOCK_PARENT);
		}
	}
	else if (!strncmp(argv[0], "processor", 9))
	{
		argc--;
		argv++;
		ret = dump_processors_info(argc, argv);
		if (ret)
			help(HELP_DUMP_PROCESSOR);
	}
	else if (!strncmp(argv[0], "rm", 2))
	{
		argc--;
		argv++;
		ret = dump_rm_info(argc, argv);
		if (ret)
		{
			DebugP_log("Invalid arguments\r\n");
			help(HELP_DUMP_RM);
		}
	}
	else if (!strcmp(argv[0], "--help"))
	{
		help(HELP_DUMP);
		return 0;
	}
	else
	{
		DebugP_log("Invalid argument %s\r\n", argv[0]);
		help(HELP_DUMP);
		return -1;
	}
	return ret;
}
