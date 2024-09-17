/*
 * Help Library for K3CONF
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

#include "help.h"
#include <stdio.h>
#include <kernel/dpl/DebugP.h>

#define HELP_CLK_SET_PARENT_URL1 "http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/pm/clocks.html#power-management-clock-frequency-configuration-example-with-mux-programming"
#define HELP_CLK_SET_PARENT_URL2 "http://downloads.ti.com/tisci/esd/latest/2_tisci_msgs/pm/clocks.html#tisci-msg-set-clock-parent"

void help(help_category cat)
{
	if (cat >= HELP_CATEGORY_MAX) {
		DebugP_log(stderr, "help called with incorrect category!!! (%d)\r\n",
			cat);
		return;
	}

	if ((cat == HELP_ALL) || (cat == HELP_USAGE)) {
		DebugP_log("\r\n");
		DebugP_log("NAME\r\n");
		DebugP_log("	k3conf - TI K3 Configuration Diagnostic Tool\r\n");
		DebugP_log("\r\n");
		DebugP_log("SYNOPSIS\r\n");
		DebugP_log("	[--version] [--help] [--cpuinfo] <command> [<args>]\r\n");
		if (cat == HELP_USAGE) {
			DebugP_log("\r\n");
			DebugP_log("	See '--help' for more information.\r\n");
			DebugP_log("\r\n");
		}
	}

	if (cat == HELP_ALL) {
		DebugP_log("\r\n");
		DebugP_log("DESCRIPTION\r\n");
		DebugP_log("	k3conf is standalone application designed to provide a quick'n easy way to \r\n"
			"	diagnose/debug/audit TI K3 architecture based processors configuration at\r\n"
			"	runtime, with no particular kernel dependency.\r\n");
		DebugP_log("	k3conf is designed to be as much platform-agnostic as possible, being able to \r\n"
			"	run on any Linux platform and easily ported to other OS.\r\n");
		DebugP_log("	Even if k3conf today focuses mainly on TISCI related functionality, it is \r\n"
			"	intended to be extended to any other area.\r\n");

		DebugP_log("\r\n");
		DebugP_log("OPTIONS\r\n");
		DebugP_log("	--help\r\n");
		DebugP_log("	    Print k3conf help.\r\n");
		DebugP_log("\r\n");
		DebugP_log("	--version\r\n");
		DebugP_log("	    Print k3conf version.\r\n");
		DebugP_log("\r\n");
		DebugP_log("	--cpuinfo\r\n");
		DebugP_log("	    Print the host processor information.\r\n");
	}

	if (cat != HELP_USAGE) {
		DebugP_log("\r\n");
		DebugP_log("COMMANDS\r\n");
	}

	if ((cat == HELP_ALL) || (cat == HELP_SHOW) ||
	    (cat == HELP_SHOW_HOST)) {
		DebugP_log("\r\n");
		DebugP_log("	show host\r\n");
		DebugP_log("		Prints all the available TISCI hosts\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SHOW) ||
	    (cat == HELP_SHOW_DEVICE)) {
		DebugP_log("\r\n");
		DebugP_log("	show device\r\n");
		DebugP_log("		Prints all the available TISCI devices\r\n");
		DebugP_log("\r\n");
		DebugP_log("	show device <dev_id>\r\n");
		DebugP_log("		Prints the corresponding device id information\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SHOW) ||
	    (cat == HELP_SHOW_CLOCK)) {
		DebugP_log("\r\n	show clock\r\n");
		DebugP_log("		Prints all the available TISCI clocks\r\n");
		DebugP_log("\r\n	show clock <dev_id>\r\n");
		DebugP_log("		Prints the clocks for corresponding device id\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SHOW) ||
	    (cat == HELP_SHOW_PROCESSOR)) {
		DebugP_log("\r\n");
		DebugP_log("	show processor\r\n");
		DebugP_log("		Prints all the available TISCI processors\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SHOW) ||
	    (cat == HELP_SHOW_RM)) {
		DebugP_log("\r\n");
		DebugP_log("	show rm\r\n");
		DebugP_log("		Prints resources managed by System firmware\r\n");
		DebugP_log("\r\n");
		DebugP_log("	show rm <dev_id>\r\n");
		DebugP_log("		Prints resources managed by System firmware for corresponding device\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DUMP) ||
	    (cat == HELP_DUMP_DEVICE)) {
		DebugP_log("\r\n");
		DebugP_log("	dump device\r\n");
		DebugP_log("		Prints device status of all the TISCI devices\r\n");
		DebugP_log("\r\n");
		DebugP_log("	dump device <dev_id>\r\n");
		DebugP_log("		Prints the corresponding device id status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DUMP) ||
	    (cat == HELP_DUMP_CLOCK)) {
		DebugP_log("\r\n");
		DebugP_log("	dump clock\r\n");
		DebugP_log("		Prints clock status all the available TISCI clocks\r\n");
		DebugP_log("\r\n");
		DebugP_log("	dump clock <dev_id>\r\n");
		DebugP_log("		Prints the available clock status for corresponding device id\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DUMP) ||
	    (cat == HELP_DUMP_CLOCK_PARENT)) {
		DebugP_log("\r\n");
		DebugP_log("	dump parent_clock <dev_id> <clk_id>\r\n");
		DebugP_log("		Prints the clock parent of provided clock\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DUMP) ||
	    (cat == HELP_DUMP_PROCESSOR)) {
		DebugP_log("\r\n");
		DebugP_log("	dump processor\r\n");
		DebugP_log("		Prints status of all the available TISCI processors\r\n");
		DebugP_log("\r\n");
		DebugP_log("	dump processor <proc_id>\r\n");
		DebugP_log("		Prints status of the given TISCI processors\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DUMP) ||
	    (cat == HELP_DUMP_RM)) {
		DebugP_log("\r\n");
		DebugP_log("	dump rm [OPTIONS]\r\n");
		DebugP_log("		Prints resource allocation for all utypes / hosts\r\n");
		DebugP_log("\r\n");
		DebugP_log("	dump rm [OPTIONS] <dev_id>\r\n");
		DebugP_log("		Prints resource allocation for corresponding device type\r\n");
		DebugP_log("\r\n");
		DebugP_log("	dump rm [OPTIONS] <dev_id> <subtype>\r\n");
		DebugP_log("		Prints resource allocation for corresponding device/type\r\n");
		DebugP_log("\r\n");
		DebugP_log("		[OPTIONS]\r\n");
		DebugP_log("		-h <host_id>\r\n");
		DebugP_log("			Filter only for corresponding host_id\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_ENABLE) ||
	    (cat == HELP_ENABLE_DEVICE)) {
		DebugP_log("\r\n");
		DebugP_log("	enable device <dev_id>\r\n");
		DebugP_log("		Enables the TISCI device and prints the status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_ENABLE) ||
	    (cat == HELP_ENABLE_CLOCK)) {
		DebugP_log("\r\n");
		DebugP_log("	enable clock <dev_id> <clk_id>\r\n");
		DebugP_log("		Enables the TISCI clock and prints the status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DISABLE) ||
	    (cat == HELP_DISABLE_DEVICE)) {
		DebugP_log("\r\n");
		DebugP_log("	disable device <dev_id>\r\n");
		DebugP_log("		Disables the TISCI device and prints the status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_DISABLE) ||
	    (cat == HELP_DISABLE_CLOCK)) {
		DebugP_log("\r\n");
		DebugP_log("	disable clock <dev_id> <clk_id>\r\n");
		DebugP_log("		Disables the TISCI clock and prints the status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SET) ||
	    (cat == HELP_SET_CLOCK)) {
		DebugP_log("\r\n");
		DebugP_log("	set clock <dev_id> <clk_id> <freq>\r\n");
		DebugP_log("		Sets the clock frequency and prints the status\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_SET) ||
	    (cat == HELP_SET_CLOCK_PARENT)) {
		DebugP_log("\r\n");
		DebugP_log("	set parent_clock <dev_id> <clk_id> <parent_clk_id>\r\n");
		DebugP_log("		Sets the parent clock for a clock mux and prints the mux status\r\n");
		DebugP_log("		Refer to the following documentation for preconditions:\r\n");
		DebugP_log("		%s\r\n", HELP_CLK_SET_PARENT_URL1);
		DebugP_log("		%s\r\n", HELP_CLK_SET_PARENT_URL2);
	}
	if ((cat == HELP_ALL) || (cat == HELP_SET) ||
	    (cat == HELP_SET_PROCESSOR)) {
		DebugP_log("\r\n");
		DebugP_log("	set processor <processor_id> <frequency>\r\n");
		DebugP_log("		Sets the processor to specified frequency\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_READ)) {
		DebugP_log("\r\n");
		DebugP_log("	read <addr> [<size>]\r\n");
		DebugP_log("		No.of bits to be read is given in the size argument\r\n");
		DebugP_log("		Expected input size is 8,16,32,64\r\n");
		DebugP_log("		Prints the value at the specified io memory\r\n");
	}
	if ((cat == HELP_ALL) || (cat == HELP_WRITE)) {
		DebugP_log("\r\n");
		DebugP_log("	write <addr> <val> [<size>]\r\n");
		DebugP_log("		No.of bits to be written is given in the size argument\r\n");
		DebugP_log("		Expected input size is 8,16,32,64\r\n");
		DebugP_log("		Writes the value at the specified io memory\r\n");
	}
}
