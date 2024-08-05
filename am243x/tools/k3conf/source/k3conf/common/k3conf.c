/*
 * K3CONF main entry file
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

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/sciclient.h>
#include "tisci.h"
#include "socinfo.h"
#include "help.h"
#include "string.h"
#include "k3conf.h"

#ifdef DEBUG
#define dprintf(format, ...)	 printf(format, ## __VA_ARGS__)
#else
#define dprintf(format, ...)
#endif

int isValidDeviceId(uint32_t dev_id)
{
	struct ti_sci_devices_info *devices = soc_info.sci_info.devices_info;
	for(int itr=0; itr<soc_info.sci_info.num_devices; itr++) 
	{
		if(devices[itr].dev_id == dev_id)
		{
			return 1;
		}
	}
	return 0;
}

int isValidProcessorId(uint32_t processor_id)
{
	struct ti_sci_processors_info *processors = soc_info.sci_info.processors_info;
	for(int itr=0; itr<soc_info.sci_info.num_processors; itr++) 
	{
		if(processors[itr].processor_id == processor_id)
		{
			return 1;
		}
	}
	return 0;
}

void k3conf_get_version(struct tisci_msg_version_req *request, struct tisci_msg_version_resp *response)
{
	/* Check for the SYSFW version by sending a request */
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) request,
        sizeof(*request),
        SystemP_WAIT_FOREVER
    };

    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) response,
        sizeof (*response)
    };

    int retVal = Sciclient_service(&reqPrm, &respPrm);
    DebugP_assert(SystemP_SUCCESS == retVal && respPrm.flags == TISCI_MSG_FLAG_ACK);
}

void k3conf_print_version()
{
	struct ti_sci_version_info version = soc_info.sci_info.version;
	char temp[100];
	char *str = NULL;
	memset(temp, '=', 95);
	strncpy(temp+95, "\r\n", 5);
	DebugP_log("%s", temp);
	str = "DMSC Firmware Version ";
    DebugP_log("| %-24s| %-65s |\r\n", str, (char *)version.firmware_description);
    str = "Firmware revision ";
	DebugP_log("| %-24s| 0x%-63x |\r\n", str, version.firmware_version);
	str = "ABI revision ";
    DebugP_log("| %-24s| %2d.%-2d %60s|\r\n", str, version.abi_major,version.abi_minor, " ");
	DebugP_log("%s", temp);
}


int show_main(int argc, char **argv)
{
	int ret = 0;

	ret = soc_init();
	if(ret)
	{
		goto main_exit;
	}

	if (argc == 0) {
		help(HELP_USAGE);
		ret = -1;
		goto main_exit;
	}

	if (!strcmp(argv[0], "--help")) {
		help(HELP_ALL);
		goto main_exit;
	}

	if (!strcmp(argv[0], "--version")) {
		k3conf_print_version();
		goto main_exit;
	}

	if (!strcmp(argv[0], "read")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_read_command(argc, argv);
	}

	if (!strcmp(argv[0], "write")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_write_command(argc, argv);
	}

	if (!strcmp(argv[0], "--cpuinfo")) {
		k3conf_print_version();
		dump_processors_info(0, NULL);
		goto main_exit;
	}

	if (!strcmp(argv[0], "show")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_show_command(argc, argv);
	}

	if (!strcmp(argv[0], "dump")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_dump_command(argc, argv);
	}

	if (!strcmp(argv[0], "enable")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_enable_command(argc, argv);
	}

	if (!strcmp(argv[0], "disable")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_disable_command(argc, argv);
	}

	if (!strcmp(argv[0], "set")) {
		argc--;
		argv++;
		k3conf_print_version();
		return process_set_command(argc, argv);
	}

	DebugP_log("Invalid argument %s\r\n", argv[0]);
	help(HELP_USAGE);
	ret = -1;
	/* Fallthrough */

main_exit:
	return ret;
}
