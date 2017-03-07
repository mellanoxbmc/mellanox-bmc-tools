/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __JTAG_HANDLERS__
#define __JTAG_HANDLERS__

#define WRITE_HANDLER_INIT_CMD	0
#define WRITE_HANDLER_BYTE_CMD	1
#define WRITE_HANDLER_SEND_CMD	2

#ifdef DISABLE_JTAG_PROG
enum aspeed_jtag_xfer_mode {
	JTAG_XFER_HW_MODE = 0,
	JTAG_XFER_SW_MODE = 1,
};

struct runtest_idle {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned char			reset;	/* Test Logic Reset */
	unsigned char			end;	/* o: idle, 1: ir pause, 2: drpause */
	unsigned char			tck;	/* keep tck */
};

struct sir_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned short			length;	/* bits */
	unsigned int			tdi;
	unsigned int			tdo;
	unsigned char			endir;	/* 0: idle, 1:pause */
};

struct sdr_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned char			direct; /* 0 ; read , 1 : write */
	unsigned short			length;	/* bits */
	unsigned int			*tdio;
	unsigned char			enddr;	/* 0: idle, 1:pause */
};

#endif

void jtag_handlers_init(void);
int null_handler(unsigned char cmd, char data);
int frequency_handler(unsigned char cmd, char data);
int runtest_handler(unsigned char cmd, char data);
int jtag_cmd_handler(unsigned char cmd, char data);

#endif /*__JTAG_HANDLERS__*/

