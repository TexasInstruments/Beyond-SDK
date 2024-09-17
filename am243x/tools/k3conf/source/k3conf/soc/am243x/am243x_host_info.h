/*
 * AM243X Host Info
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - https://www.ti.com/
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

#ifndef __AM243X_HOST_INFO_H
#define __AM243X_HOST_INFO_H

#define AM243X_HOST_ID_DMSC	0
#define AM243X_HOST_ID_MAIN_0_R5_0	35
#define AM243X_HOST_ID_MAIN_0_R5_1	36
#define AM243X_HOST_ID_MAIN_0_R5_2	37
#define AM243X_HOST_ID_MAIN_0_R5_3	38
#define AM243X_HOST_ID_M4_0	30
#define AM243X_HOST_ID_MAIN_1_R5_0	40
#define AM243X_HOST_ID_MAIN_1_R5_1	41
#define AM243X_HOST_ID_MAIN_1_R5_2	42
#define AM243X_HOST_ID_MAIN_1_R5_3	43
#define AM243X_HOST_ID_ICSSG_0	50
#define AM243X_HOST_ID_ICSSG_1	51

#define AM243X_MAX_HOST_IDS	12

extern struct ti_sci_host_info am243x_host_info[];

#endif /* __AM243X_HOST_INFO_H */
