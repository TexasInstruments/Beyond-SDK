/*
 * Help Library Header File for K3CONF
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

#ifndef __K3CONF_HELP
#define __K3CONF_HELP

typedef enum {
	HELP_USAGE,
	HELP_SHOW,
	HELP_SHOW_HOST,
	HELP_SHOW_DEVICE,
	HELP_SHOW_CLOCK,
	HELP_SHOW_PROCESSOR,
	HELP_SHOW_RM,
	HELP_DUMP,
	HELP_DUMP_DEVICE,
	HELP_DUMP_CLOCK,
	HELP_DUMP_CLOCK_PARENT,
	HELP_DUMP_PROCESSOR,
	HELP_DUMP_RM,
	HELP_ENABLE,
	HELP_ENABLE_DEVICE,
	HELP_ENABLE_CLOCK,
	HELP_DISABLE,
	HELP_DISABLE_DEVICE,
	HELP_DISABLE_CLOCK,
	HELP_SET,
	HELP_SET_CLOCK,
	HELP_SET_CLOCK_PARENT,
	HELP_SET_PROCESSOR,
	HELP_READ,
	HELP_WRITE,
	HELP_ALL,
	HELP_CATEGORY_MAX,
} help_category;

void help(help_category cat);
#endif
