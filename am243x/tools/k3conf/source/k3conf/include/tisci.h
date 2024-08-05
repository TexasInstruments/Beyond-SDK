/*
 * TISCI helper apis header file
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - https://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
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

#ifndef __TISCI_H
#define __TISCI_H

#include <stdint.h>

struct ti_sci_version_info {
	uint8_t abi_major;
	uint8_t abi_minor;
	uint16_t firmware_version;
	char firmware_description[32];
};

struct ti_sci_host_info {
	uint32_t host_id;
	char host_name[15];
	char security_status[15];
	char description[50];
};

struct ti_sci_processors_info {
	uint32_t processor_id;
	uint32_t dev_id;
	char name[30];
};

struct ti_sci_devices_info {
	uint32_t dev_id;
	char name[60];
};

struct ti_sci_clocks_info {
	uint32_t dev_id;
	uint32_t clk_id;
	char clk_name[100];
	char clk_function[100];
};

struct ti_sci_rm_info {
	uint32_t utype;
	uint32_t dev_id;
	char device_name[100];
	uint32_t subtype_id;
	char subtype_name[100];
};

struct ti_sci_info {
	uint8_t host_id;
	struct ti_sci_version_info version;
	struct ti_sci_host_info *host_info;
	uint32_t num_hosts;
	struct ti_sci_processors_info *processors_info;
	uint32_t num_processors;
	struct ti_sci_devices_info *devices_info;
	uint32_t num_devices;
	struct ti_sci_clocks_info *clocks_info;
	uint32_t num_clocks;
	struct ti_sci_rm_info *rm_info;
	uint32_t num_res;
};

struct ti_sci_rm_desc {
	uint16_t start;
	uint16_t num;
	uint16_t start_sec;
	uint16_t num_sec;
};

#endif
