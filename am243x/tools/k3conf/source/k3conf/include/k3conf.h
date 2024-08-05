/*
 * K3CONF Main Header file.
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

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "socinfo.h"
#include <string.h>
#include <drivers/sciclient.h>

#ifndef __K3CONF_H
#define __K3CONF_H

#define INVALID_NUM 0xFFFFFFFF

int show_main(int argc, char **argv);
int isValidDeviceId(uint32_t dev_id);
int isValidProcessorId(uint32_t dev_id);
void k3conf_get_version(struct tisci_msg_version_req *request, struct tisci_msg_version_resp *response);
int process_show_command(int argc, char *argv[]);
int process_dump_command(int argc, char *argv[]);
int dump_clocks_info(int argc, char *argv[]);
int dump_clock_parent_info(int argc, char *argv[]);
int dump_devices_info(int argc, char *argv[]);
int dump_processors_info(int argc, char *argv[]);
int process_enable_command(int argc, char *argv[]);
int process_disable_command(int argc, char *argv[]);
int process_set_command(int argc, char *argv[]);
int process_read_command(int argc, char *argv[]);
int process_write_command(int argc, char *argv[]);
#endif
