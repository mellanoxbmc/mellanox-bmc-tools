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



#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <fcntl.h>
#include <time.h>
#ifndef DISABLE_JTAG_PROG
#include <uapi/linux/aspeed_jtag.h>
#include <uapi/linux/ioctl.h>
#endif
#include "vmopcode.h"
#include "utilities.h"
#include "jtag_handlers.h"

#define JTAG_DEBUG_LEVEL 0
typedef struct {
	char cmd;
	enum jtag_data_state_e{
		JTAG_IDLE,
		JTAG_CMD,
		JTAG_TOKEN,
		JTAG_BYTE,
		JTAG_ERR
	} state;

	unsigned int bit_size;
	char *tdi;
	char *tdo;
	char *mask;

	char *wr_data_p;
	unsigned int data_pos;
	char size_shift;
} jtag_handler_data_t;


typedef struct {


} frq_handler_data_t;

typedef struct {
	enum runtest_state_e{
		RUNTEST_IDLE,
		RUNTEST_VAL,
		RUNTEST_ERR,
	} state;
	char cmd;
	int data;
	char data_shift;

	char new_state;
	int  wait;
	int  tck;
	char end_state;
} runtest_handler_data_t;

typedef struct {
	unsigned int bit_size;
	unsigned int mask_bit_size;
	char *tdi;
	char *tdo;
	char *mask;
} jtag_transaction_t;

typedef struct {
	jtag_handler_data_t sir_sdr_data;
	frq_handler_data_t  frq_data;
	runtest_handler_data_t runtest_data;
} write_handler_data_t;

#define HIR_TRAILER	0
#define HDR_TRAILER	1
#define TIR_TRAILER	2
#define TDR_TRAILER	3
#define SIR_DATA_TR	4
#define SDR_DATA_TR	5

jtag_transaction_t g_transaction_data[6];

static char g_bitbuf[256];
static unsigned int g_bitbuf_pos = 0;

char *write_handler_cmd_str[] = {"WRITE_HANDLER_INIT_CMD",
				"WRITE_HANDLER_BYTE_CMD",
				"WRITE_HANDLER_SEND_CMD"};

static struct scanToken
{
	char * text;
	int token;
} scanTokens[] =
{
	/*********************************************************************
	*
	* Supported SVF keywords.
	*
	*********************************************************************/

	{ ";", ENDDATA },
	{ "SIR", SIR }, { "SDR", SDR },
	{ "TDI", TDI }, { "TDO", TDO }, { "MASK", MASK },
	{ "STATE", STATE },
	{ "TCK", TCK, }, { "WAIT", WAIT },
	{ "XSDR", XSDR }, { "XTDI", XTDI }, { "XTDO", XTDO },
	{ "ENDDR", ENDDR },
	{ "ENDIR", ENDIR },
	{ "HIR", HIR }, { "TIR", TIR }, { "HDR", HDR }, { "TDR", TDR },
	{ "MEM", MEM },
	{ "RUNTEST", RUNTEST },
	{ "ENDSTATE", ENDSTATE },
	{ "TRST", TRST },
	{ "FREQUENCY", FREQUENCY },
	{ "SEC", SEC },
	{ "SMASK", SMASK },
	{ "MAXIMUM", MAX },
	{"ON", ON},{"OFF", OFF},{"ISPEN",ispEN}, {"HIGH", HIGH},{"LOW", LOW},
	{ "SETFLOW", SETFLOW }, { "RESETFLOW", RESETFLOW },
	{ "REPEAT", REPEAT }, { "ENDLOOP", ENDLOOP },
	{ "(", LEFTPAREN },
	{ "CRC", CRC },
	{ "CMASK", CMASK },
	{ "RMASK", RMASK },
	{ "READ", READ },
	{ "DMASK", DMASK },
	{ "VUES",  VUES },
	{ "LCOUNT", LCOUNT },
	{ "LDELAY", LDELAY },
	{ "LSDR", LSDR },
	{ "LVDS", LVDS },
	{ "LOOP", LOOP }
};

static int ScanTokenMax = sizeof( scanTokens ) / sizeof( scanTokens[ 0 ] );

extern int g_JTAGFile;
extern char g_direct_prog;
unsigned short g_usCpu_Frequency  = 300;   /*Enter your CPU frequency here, unit in MHz.*/
write_handler_data_t g_write_handler_data;

char * get_token_str(char token){
	int i;
	for (i=0; i<ScanTokenMax; i++){
		if (scanTokens[i].token ==  token){
			return scanTokens[i].text;
		}
	}
	return "Unknown Token";
}

void jtag_print_xfer(jtag_transaction_t * data_p, int more_data){
	int i;

	if (data_p->bit_size == 0)
		return;

	if (data_p->tdi){
		printf("TDI %d  (", data_p->bit_size);
		for (i = data_p->bit_size; i > 0; i -= 8)
			printf("%02X", (unsigned char)data_p->tdi[(i-1) / 8]);
		printf(")\n");
	}
	if (data_p->tdo){
		printf("TDO %d  (", data_p->bit_size);
		for (i = data_p->bit_size; i > 0; i -= 8)
			printf("%02X", (unsigned char)data_p->tdo[(i-1) / 8]);
		printf(")\n");

		if ((data_p->mask) && (data_p->mask_bit_size == data_p->bit_size)){
			printf("MASK %d (", data_p->bit_size);
			for (i = data_p->bit_size; i > 0; i -= 8)
				printf("%02X", (unsigned char)data_p->mask[(i-1) / 8]);
			printf(")\n");
		}
	}
}

static int char2int(char ** data_p, int bit_size){
	int size;
	int data_o=0;

	size = (bit_size+7) / 8;
	memcpy(&data_o, *data_p, size);
	(*data_p)+=size;
	return data_o;
}

static void put_bitbuffer(char *data, unsigned int bit_size){
	unsigned int bit_pos = 0;
	unsigned char bit_offset = 0;
	unsigned char byte_offset = 0;
	unsigned char data_bit_offset = 0;

	byte_offset = g_bitbuf_pos / 8;
	bit_offset = g_bitbuf_pos % 8;
	for (bit_pos = 0; bit_pos < bit_size; bit_pos++)
	{
		g_bitbuf[byte_offset] &= ~(1<<bit_offset);
		g_bitbuf[byte_offset] |= *data & (1<<data_bit_offset) ? (1<<bit_offset) : 0;
		bit_offset++;
		if (bit_offset == 8){
			bit_offset = 0;
			byte_offset ++;
		}

		data_bit_offset++;
		if (data_bit_offset == 8){
			data_bit_offset = 0;
			data++;
		}
		g_bitbuf_pos++;
	}
}

static void merge_bitbuffer( char *head, int head_len,
							char *data, int data_len,
							char *tail, int tail_len)
{
	g_bitbuf_pos = 0;
	memset(g_bitbuf, 0, sizeof(g_bitbuf));

	put_bitbuffer(head, head_len);
	put_bitbuffer(data, data_len);
	put_bitbuffer(tail, tail_len);
}

static void extract_bitbuffer(char *in_buf, int inbuf_len,
							int head_len,
							char *data, int data_len,
							int tail_len)
{
	unsigned int bit_pos = 0;
	unsigned char bit_offset = 0;	/* bit pos in input data*/
	unsigned char byte_offset = 0;	/* byte pos in input data*/
	unsigned char data_bit_offset = 0; /*bit pos in output_data*/

	/* HDR */
	bit_pos += head_len;
	bit_offset = head_len;
	byte_offset = bit_offset/8;
	bit_offset %= 8;
	data_bit_offset = 0;
	in_buf += byte_offset;

	/* SDR */
	while (bit_pos < (inbuf_len - tail_len))
	{
		*data &= ~(1<<data_bit_offset);
		*data |= *in_buf & (1<<bit_offset) ? (1<<data_bit_offset) : 0;
		data_bit_offset++;
		if (data_bit_offset == 8){
			data_bit_offset = 0;
			data++;
		}
		bit_offset++;
		if (bit_offset == 8){
			bit_offset = 0;
			in_buf++;
		}
		bit_pos++;
	}
}

static int jtag_sir_xfer(void)
{
	struct aspeed_jtag_sir_xfer xfer;
	char *mask_p = NULL;
	char *tdo_p = NULL;
	int TDO_expected = 0;
	int bit_remaining = 0;
	int MASK_data = 0xffffffff;
	char CurBit = 0;
	int i;
	int ret = 0;
	int tdo_data_buf[16];	/*buffer to store received tdo data*/
	int *tdo_data;

	memset(&xfer, 0 ,sizeof(xfer));
#if JTAG_DEBUG_LEVEL > 0
	printf("JTAG SIR_CMD\n");
	jtag_print_xfer(&g_transaction_data[HIR_TRAILER], 0);
	jtag_print_xfer(&g_transaction_data[SIR_DATA_TR], 0);
	jtag_print_xfer(&g_transaction_data[TIR_TRAILER], 0);
#endif

	merge_bitbuffer(g_transaction_data[HIR_TRAILER].tdi, g_transaction_data[HIR_TRAILER].bit_size,
						g_transaction_data[SIR_DATA_TR].tdi, g_transaction_data[SIR_DATA_TR].bit_size,
						g_transaction_data[TIR_TRAILER].tdi, g_transaction_data[TIR_TRAILER].bit_size);

	tdo_p = g_transaction_data[SIR_DATA_TR].tdo;
	mask_p = g_transaction_data[SIR_DATA_TR].mask;

	xfer.mode = ASPEED_JTAG_XFER_SW_MODE;
	xfer.tdi = ((unsigned int*)g_bitbuf)[0];
	xfer.length = g_bitbuf_pos;

#ifndef DISABLE_JTAG_PROG
	xfer.endir = ASPEED_JTAG_STATE_IDLE;
	ioctl(g_JTAGFile, ASPEED_JTAG_IOCSIR, &xfer);
#endif

	/* check tdo */
	if (tdo_p){
		tdo_data = tdo_data_buf;

		extract_bitbuffer((char *)&xfer.tdo, xfer.length,
						g_transaction_data[HIR_TRAILER].bit_size,
						(char *)tdo_data,
						g_transaction_data[SIR_DATA_TR].bit_size,
						g_transaction_data[TIR_TRAILER].bit_size);

		bit_remaining = g_transaction_data[SIR_DATA_TR].bit_size;

		/*check mask if exists*/
		MASK_data = 0xffffffff;
		if ((mask_p) && (g_transaction_data[SIR_DATA_TR].mask_bit_size == g_transaction_data[SIR_DATA_TR].bit_size)){
			MASK_data = char2int(&mask_p, bit_remaining);
		}

		TDO_expected = char2int(&tdo_p, bit_remaining);

#if JTAG_DEBUG_LEVEL > 1
		printf("SIR Check mask TDO_real 0x%08x MASK 0x%08x TDO_expect 0x%08x\n",
				*tdo_data, MASK_data, TDO_expected);
#endif
		for (i = 0; i < bit_remaining; i++) {
			CurBit = *tdo_data & (1 << i) ? 1 : 0;

			if (MASK_data & (1<<i)) {
				if (CurBit != (TDO_expected & (1 << i) ? 1 : 0)){
					ret = -1;
				}
			}
		}

	}
	return ret;
}

static int jtag_sdr_xfer(void)
{
	struct aspeed_jtag_sdr_xfer xfer;
	char *mask_p = NULL;
	char *tdo_p = NULL;
	int TDO_expected = 0;
	int bit_pos = 0;
	int bit_remaining = 0;
	int MASK_data = 0xffffffff;
	char CurBit = 0;
	int i;
	int ret = 0;
	int tdo_data_buf[64];	/*buffer to store received tdo data*/
	int *tdo_data;

#if JTAG_DEBUG_LEVEL > 0
	printf("JTAG SDR_CMD\n");
	jtag_print_xfer(&g_transaction_data[HDR_TRAILER], 0);
	jtag_print_xfer(&g_transaction_data[SDR_DATA_TR], 0);
	jtag_print_xfer(&g_transaction_data[TDR_TRAILER], 0);
#endif
	memset(&xfer, 0 ,sizeof(xfer));

	merge_bitbuffer(g_transaction_data[HDR_TRAILER].tdi, g_transaction_data[HDR_TRAILER].bit_size,
						g_transaction_data[SDR_DATA_TR].tdi, g_transaction_data[SDR_DATA_TR].bit_size,
						g_transaction_data[TDR_TRAILER].tdi, g_transaction_data[TDR_TRAILER].bit_size);

	tdo_p = g_transaction_data[SDR_DATA_TR].tdo;
	mask_p = g_transaction_data[SDR_DATA_TR].mask;

	xfer.mode = ASPEED_JTAG_XFER_SW_MODE;
	xfer.tdio = (unsigned int*)g_bitbuf;
	xfer.length = g_bitbuf_pos;

	if (tdo_p)
		xfer.direct = 0;
	else
		xfer.direct = 1;

#ifndef DISABLE_JTAG_PROG
	xfer.enddr = ASPEED_JTAG_STATE_IDLE;
	ioctl(g_JTAGFile, ASPEED_JTAG_IOCSDR, &xfer);
#endif

	/* check tdo */
	if (tdo_p){
		tdo_data = tdo_data_buf;

		extract_bitbuffer(	(char *)xfer.tdio, xfer.length,
							g_transaction_data[HDR_TRAILER].bit_size,
							(char *)tdo_data,
							g_transaction_data[SDR_DATA_TR].bit_size,
							g_transaction_data[TDR_TRAILER].bit_size);

		bit_pos = 0;
		while (bit_pos < g_transaction_data[SDR_DATA_TR].bit_size) {
			bit_remaining = g_transaction_data[SDR_DATA_TR].bit_size - bit_pos;
			if (bit_remaining > 32) {
				bit_remaining = 32;
				bit_pos += 32;
			} else {
				bit_pos += bit_remaining;
			}

			/*check mask if exists*/
			MASK_data = 0xffffffff;
			if ((mask_p) && (g_transaction_data[SDR_DATA_TR].mask_bit_size == g_transaction_data[SDR_DATA_TR].bit_size)){
				MASK_data = char2int(&mask_p, bit_remaining);
			}

			TDO_expected = char2int(&tdo_p, bit_remaining);

#if JTAG_DEBUG_LEVEL > 1
			printf("SDR Check mask TDO_real 0x%08x MASK 0x%08x TDO_expect 0x%08x\n",
					*tdo_data, MASK_data, TDO_expected);
#endif

			for (i = 0; i < bit_remaining; i++) {
				CurBit = *tdo_data & (1 << i) ? 1 : 0;

				if (MASK_data & (1<<i)) {
					if (CurBit != (TDO_expected & (1 << i) ? 1 : 0)){
						ret = -1;
					}
				}
			}
			tdo_data++;
		}
		}
	return ret;
}

static int jtag_set_transaction_data(jtag_handler_data_t * data_p,
									unsigned char type)
{
	jtag_transaction_t *transacrtion_data_p = &g_transaction_data[type];
	unsigned int size = 0;

	if (transacrtion_data_p->tdi){
		free(transacrtion_data_p->tdi);
		transacrtion_data_p->tdi = NULL;
	}
	if (transacrtion_data_p->tdo){
		free(transacrtion_data_p->tdo);
		transacrtion_data_p->tdo = NULL;
	}

	if (data_p->bit_size){
		size = (data_p->bit_size+7)/8;

		if(data_p->tdi){
			transacrtion_data_p->tdi = malloc(size);
			memcpy(transacrtion_data_p->tdi, data_p->tdi, size);
		}
		if (data_p->tdo) {
			transacrtion_data_p->tdo = malloc(size);
			memcpy(transacrtion_data_p->tdo, data_p->tdo, size);
		}
		if (data_p->mask) {
			if (transacrtion_data_p->mask)
				free(transacrtion_data_p->mask);
			transacrtion_data_p->mask = malloc(size);
			memcpy(transacrtion_data_p->mask, data_p->mask, size);
			transacrtion_data_p->mask_bit_size = data_p->bit_size;
		}
	}
	transacrtion_data_p->bit_size = data_p->bit_size;
	return 0;
}

static int jtag_runtest_xfer(runtest_handler_data_t * data_p)
{
	struct aspeed_jtag_runtest_idle runtest;
	unsigned short delay = 0;
	unsigned short loop_index = 0;
	unsigned short ms_index = 0;
	unsigned short us_index = 0;
	unsigned short g_usCpu_Frequency = 150;

#if JTAG_DEBUG_LEVEL > 0
	printf("RUNTEST_CMD\n");
	if (data_p->new_state != (char)0xff)
		printf("State:%d\n", data_p->new_state);

	if (data_p->end_state != (char)0xff)
		printf("End state:%d\n", data_p->end_state);

	printf("TCK:%d\n", data_p->tck);
#endif

	if (data_p->tck){
		runtest.mode = ASPEED_JTAG_XFER_SW_MODE;
		runtest.end = 0;	/*IDLE*/
		runtest.reset = 0;
		runtest.tck = data_p->tck;
#ifndef DISABLE_JTAG_PROG
		ioctl(g_JTAGFile, ASPEED_JTAG_IOCRUNTEST, &runtest);
#endif
	}

	if (data_p->wait){
		delay = data_p->wait;

		if ( delay & 0x8000 ) /*Test for unit*/
		{
			delay  &= ~0x8000; /*unit in milliseconds*/
		} else { /*unit in microseconds*/
			delay  = (unsigned short) (delay/1000); /*convert to milliseconds*/
			if ( delay <= 0 ) {
				delay  = 1; /*delay is 1 millisecond minimum*/
			}
		}
#if JTAG_DEBUG_LEVEL > 0
		printf("WAIT %d ms\n", delay);
#endif
		/*Users can replace the following section of code by their own*/
			for( ms_index = 0; ms_index < delay; ms_index++)
			{
				/*Loop 1000 times to produce the milliseconds delay*/
				for (us_index = 0; us_index < 1000; us_index++)
				{ /*each loop should delay for 1 microsecond or more.*/
					loop_index = 0;
					do {
						/*The NOP fakes the optimizer out so that it doesn't toss out the loop code entirely*/
						asm("NOP");
					}while (loop_index++ < ((g_usCpu_Frequency/8)+(+ ((g_usCpu_Frequency % 8) ? 1 : 0))));
				}
			}

	}

	return 0;
}

int jtag_cmd_handler(unsigned char cmd, char data)
{
	int ret = 0;
	jtag_handler_data_t * data_p;

	if (!g_direct_prog)
		return 0;

	data_p = &g_write_handler_data.sir_sdr_data;

#if JTAG_DEBUG_LEVEL > 2
	printf(">jtag_cmd_handler(%s) %x\n", write_handler_cmd_str[cmd], data);
#endif
	if (cmd == WRITE_HANDLER_INIT_CMD){
		memset(data_p, 0, sizeof(*data_p));
		data_p->cmd = data; /* SDR, SDR, HIR, HDR, TIR, TDR */
		return 0;
	} else if (cmd == WRITE_HANDLER_SEND_CMD){
		switch (data_p->cmd){
			case SIR:
				jtag_set_transaction_data(data_p, SIR_DATA_TR);
				ret = jtag_sir_xfer();
				break;
			case SDR:
				jtag_set_transaction_data(data_p, SDR_DATA_TR);
				ret = jtag_sdr_xfer();
				break;
			case HIR:
				jtag_set_transaction_data(data_p, HIR_TRAILER);
				break;
			case HDR:
				jtag_set_transaction_data(data_p, HDR_TRAILER);
				break;
			case TIR:
				jtag_set_transaction_data(data_p, TIR_TRAILER);
				break;
			case TDR:
				jtag_set_transaction_data(data_p, TDR_TRAILER);
				break;
			default:
				ret = -1;
				goto cleanup;
		}
		goto cleanup;
	}

	switch (data_p->state){
		case JTAG_IDLE:
#if JTAG_DEBUG_LEVEL > 2
			printf("state:JTAG_IDLE\n");
#endif
			/* wait for opcode data*/
			if (data == data_p->cmd)
				data_p->state = JTAG_CMD;

			break;
		case JTAG_CMD:
#if JTAG_DEBUG_LEVEL > 2
			printf("state:JATG_CMD\n");
#endif
			/* receive num bites */

			data_p->bit_size |= (data & 0x007f) << data_p->size_shift;
			if (data & 0x0080){
				data_p->size_shift+=7;
			} else {
				data_p->state = JTAG_TOKEN;
			}
#if JTAG_DEBUG_LEVEL > 2
			printf("size:%d\n", data_p->bit_size);
#endif
			break;
		case JTAG_TOKEN:
#if JTAG_DEBUG_LEVEL > 2
			printf("state:JTAG_TOKEN -> %s\n", get_token_str(data));
#endif
			data_p->state = JTAG_BYTE;
			data_p->data_pos = 0;

			switch (data){
				case TDI:
					data_p->tdi = malloc((data_p->bit_size+7)/8);
					if (!data_p->tdi)
						goto cleanup;
					data_p->wr_data_p = data_p->tdi;
					break;
				case TDO:
					data_p->tdo = malloc((data_p->bit_size+7)/8);
					if (!data_p->tdo)
						goto cleanup;
					data_p->wr_data_p = data_p->tdo;
					break;
				case MASK:
					data_p->mask = malloc((data_p->bit_size+7)/8);
					if (!data_p->mask)
						goto cleanup;
					data_p->wr_data_p = data_p->mask;
					break;
				case CONTINUE:
				case SMASK:
				case CMASK:
				case RMASK:
				case DMASK:
				case CRC:
				case READ:
				case ENDDATA:
					break;
				default:
					data_p->state = JTAG_ERR;
					ret = -1;
					break;
			}
			break;
		case JTAG_BYTE:
			*data_p->wr_data_p++ = reverse_bits(data);
			data_p->data_pos += 8;

			if (data_p->data_pos >= data_p->bit_size){
#if JTAG_DEBUG_LEVEL > 2
				printf("data_pos:%d size:%d\n", data_p->data_pos, data_p->bit_size);
#endif
				data_p->state = JTAG_TOKEN;
			}
			break;
		case JTAG_ERR:
			ret = -1;
			break;
		default:
			ret = -1;
			goto cleanup;
			break;
	}

	return ret;

cleanup:
	data_p->state = JTAG_IDLE;
	if (data_p->tdi)
		free(data_p->tdi);

	if (data_p->tdo)
		free(data_p->tdo);

	if (data_p->mask)
		free(data_p->mask);
	memset(data_p, 0 ,sizeof(*data_p));
	return ret;
}

int null_handler(unsigned char cmd, char data)
{
	if (!g_direct_prog)
		return 0;

	return 0;
}

int frequency_handler(unsigned char cmd, char data)
{
	if (!g_direct_prog)
		return 0;

	return 0;
}

int runtest_handler(unsigned char cmd, char data)
{
	int ret = 0;
	runtest_handler_data_t * data_p;

	if (!g_direct_prog)
		return 0;

	data_p = &g_write_handler_data.runtest_data;
#if JTAG_DEBUG_LEVEL > 2
	printf(">runtest_handler(%d) %x\n", cmd, data);
#endif
	if (cmd == WRITE_HANDLER_INIT_CMD){
		memset(data_p, 0, sizeof(*data_p));
		data_p->state = RUNTEST_IDLE;
		data_p->new_state = 0xff;
		data_p->end_state = 0xff;
		return ret;
	} else if (cmd == WRITE_HANDLER_SEND_CMD){
		jtag_runtest_xfer(data_p);
		return ret;
	}

	switch (data_p->state){
		case RUNTEST_IDLE:
			data_p->cmd = data;
			data_p->data = 0;
			data_p->data_shift = 0;
			data_p->state = RUNTEST_VAL;
			break;
		case RUNTEST_VAL:
			data_p->data |= (data & 0x007f) << data_p->data_shift;
			if (data & 0x0080){
				data_p->data_shift+=7;
			} else {
				data_p->state = RUNTEST_IDLE;

				switch (data_p->cmd){
					case STATE:
						if (data_p->new_state == (char)0xff) /*check for ENDSTATE command*/
							data_p->new_state = data_p->data;
						else
							data_p->end_state = data_p->data;
						break;
					case TCK:
						data_p->tck +=data_p->data;
						break;
					case WAIT:
						data_p->wait +=data_p->data;
						break;
					default:
						data_p->state = RUNTEST_ERR;
						ret = -1;
						break;
				}
			}
#if JTAG_DEBUG_LEVEL > 2
			printf("data: %d\n", data_p->data);
#endif
			break;
		case RUNTEST_ERR:
			ret = -1;
			break;
		default:
			data_p->state = RUNTEST_ERR;
			ret = -1;
			break;
	}

	return ret;
}

void jtag_handlers_init(void)
{
	memset(&g_transaction_data, 0 ,sizeof(g_transaction_data));
}
