/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2012-2015 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called license.txt.

  Contact Information:
    Intel Corporation
    2200 Mission College Blvd.
    Santa Clara, CA  97052

  BSD LICENSE

  Copyright(c) 2012-2015 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#if !defined(__KERNEL__)

#include "sventx.h"

#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/user.h>
#include <asm-generic/types.h>

/** Linux user mode SVENTX adaptation
 *
 * This file provides the SVENTX adaptation to the northpeak kernel module.
 * This module is responsible for providing direct memory access to the
 * STH MMIO area through the STH device node.
 *
 * The memory is provided as 4K sized pages. Each page contains 64 STH
 * channels, which consume 64 bytes IO-space each.
 *
 * The STH channels are managed as a double linked list of
 * @ref sth_channel_block nodes. The pthread_mutex_t state_mutex is used
 * to synchronize accesses to the block list.
 *
 * [SVENSTATE].page_block_chain
 *                  |
 *          |--------
 *          v
 *   +--------------+              +--------------+
 *   |  BLOCK #0    |  --[next]--> |  BLOCK #1    |  ...
 *   | channels[64] | <--[prev]--  | channels[64] |  ...
 *   +--------------+              +--------------+
 *
 * The library holds a single open file handle to the driver during its
 * entire life time. Errors accessing the driver are silently ignored.
 * They only cause the trace to get lost. Setting the environment variable
 *
 *    SVENTX_DEBUG=1
 *
 * enables IO error message generation to stderr to support trace generation
 * problem analysis.
 */

static const char STH_DEVICE_NAME[] = "/dev/0-sth";

#define STH_POLICY "user"
#define STH_POLICY_SZ sizeof(STH_POLICY)

/* { TODO: use linux header when merged upstream */
/* #include <linux/stm.h> */
/**
 * struct stp_policy_id - identification for the STP policy
 * @size:	size of the structure including real id[] length
 * @master:	assigned master
 * @channel:	first assigned channel
 * @width:	number of requested channels
 * @id:		identification string
 *
 * User must calculate the total size of the structure and put it into
 * @size field, fill out the @id and desired @width. In return, kernel
 * fills out @master, @channel and @width.
 */
struct stp_policy_id {
	__u32		size;
	__u16		master;
	__u16		channel;
	__u16		width;
	/* padding */
	__u16		__reserved_0;
	__u32		__reserved_1;
	char		id[0];
};

#define STP_POLICY_ID_SET	_IOWR('%', 0, struct stp_policy_id)
#define STP_POLICY_ID_GET	_IOR('%', 1, struct stp_policy_id)
/* } TODO */

static pthread_mutex_t state_mutex;

/** Structure for managing channels in a 4K STH MMIO block
 */
typedef struct s_sth_channel_block {
	struct s_sth_channel_block *next;
	struct s_sth_channel_block *prev;

#define BLOCK_FREE_MASK 0xFFFFFFFFFFFFFFFFull
	sven_u64_t free_bits;

#define STH_CHANNELS_PER_PAGE 64
	struct sven_sth_channel *channel[STH_CHANNELS_PER_PAGE];

} sth_channel_block, *psth_channel_block;

#if __GNUC__ < 3 || (__GNUC__  == 3 && __GNUC_MINOR__ < 4)
#warning(old gcc version, using workaround for missing __builtin_clzll)
/* Workaround for missing builtin __builtin_clzll() in very old GCC versions.
 * count leading zeros in 64bit value
 */
int __builtin_clzll(unsigned long long val)
{
	unsigned int dw;
	int cnt;

	/* upper 32 bit */
	dw = (unsigned int) (val >> 32);

	for (cnt = 0; cnt < 32; ++cnt) {
		if (dw & 0x80000000)
			return cnt;
		dw <<= 1;
	}

	/* lower 32 bit */
	dw = (unsigned int) val;

	for (cnt = 32; cnt < 64; ++cnt) {
		if (dw & 0x80000000)
			return cnt;
		dw <<= 1;
	}

	return 64;

}

#endif

#if defined(SVENTX_DEBUG)
/* debug helpers, only used if compiled in debug mode */

static sven_u64_t t0_tick;

/* return microsecond accurate counter */
static sven_u64_t get_tick_count_us()
{
	struct timeval tv = { 0 };
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000000) + tv.tv_usec;
}

static FILE *fp_debug;
static void print_debug(const char *fmt, ...)
{
	if (NULL != fp_debug) {
		sven_u64_t now;
		va_list args;
		va_start(args, fmt);

		/* get time in us since first printout */
		if (!t0_tick)
			t0_tick = get_tick_count_us();
		now = get_tick_count_us() - t0_tick;

		fprintf(fp_debug, "%10llu.%03llu :", now / 1000ull,
			now % 1000ull);
		vfprintf(fp_debug, fmt, args);
		va_end(args);
	}
}

#define DEBUG_OUT(fmt, ...) print_debug(fmt, __VA_ARGS__)
#else
#define DEBUG_OUT(fmt, ...)
#endif

#define PRINT_ERROR_IF(cond, fmt, ...)\
	do {\
		if (cond)\
			print_error(fmt, __VA_ARGS__);\
		DEBUG_OUT(fmt, __VA_ARGS__);\
	} while (0)

/** Internal helper for printing error messages
 */
static void print_error(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	fprintf(stderr, "SVENTX-ERROR: ");
	vfprintf(stderr, fmt, args);
	va_end(args);
}

/** Allocate a new STH channel block from the NPK driver
 * @param diag_on enable error message print out if != 0
 * @return pointer to new STH channel block or NULL on erro
 */
static psth_channel_block request_sth_block(int diag_on)
{
	psth_channel_block block;
	struct sven_sth_channel *sth_mmio;
	struct stp_policy_id *cfg = NULL;
	size_t cfg_sz;
	int i;
	int fd;

	/* allocate new block
	 */
	block = (psth_channel_block) calloc(1, sizeof(*block));

	if (NULL == block) {
		PRINT_ERROR_IF(diag_on, "error allocating %d bytes : %s\n",
			       sizeof(*block), strerror(errno));
		return NULL;
	}

	block->free_bits = BLOCK_FREE_MASK;	/* all channels unused */

#if defined(SVEN_UNIT_TEST)

	sth_mmio = (struct sven_sth_channel *) block;

#else
	/* Open the NPK device driver
	 */
	fd = open(STH_DEVICE_NAME, O_RDWR);
	if (-1 == fd) {
		PRINT_ERROR_IF(diag_on,
			       "unable to open trace device '%s' : %s\n",
			       STH_DEVICE_NAME, strerror(errno));
		free(block);
		return NULL;
	}

	/* Allocate a channel block from the NPK driver.
	 */
	cfg_sz = sizeof(*cfg) + STH_POLICY_SZ;
	cfg = calloc(1, cfg_sz);
	if (NULL == cfg) {
		PRINT_ERROR_IF(diag_on,
			       "error allocating config %d bytes : %s\n",
			       cfg_sz, strerror(errno));
		free(block);
		close(fd);
		return NULL;
	}

	cfg->size  = cfg_sz;
	cfg->width = STH_CHANNELS_PER_PAGE;
	strcpy(cfg->id, STH_POLICY);
	if (-1 == ioctl(fd, STP_POLICY_ID_SET, cfg)) {
		PRINT_ERROR_IF(diag_on,
			       "STH ioctl %08x failed : %s\n",
			       STP_POLICY_ID_SET, strerror(errno));
		free(cfg);
		free(block);
		close(fd);
		return NULL;
	}

	/* Map STH memory using the selected policy.
	 */
	sth_mmio = (struct sven_sth_channel *)
	    mmap(NULL,
		 PAGE_SIZE,
		 PROT_READ | PROT_WRITE, MAP_SHARED,
		 fd, 0);

	if (MAP_FAILED == sth_mmio) {
		PRINT_ERROR_IF(diag_on,
			       "unable to map STH channel : %s\n",
			       strerror(errno));

		free(cfg);
		free(block);
		close(fd);
		return NULL;
	}

	DEBUG_OUT
	    ("request_sth_block() block=%p, MMIO=%p\n",
	     block, sth_mmio);


	/* Close the file; munmap will release resources later.
	 */
	if (0 != close(fd)) {
		PRINT_ERROR_IF(diag_on,
			       "unable to close trace device handle %d : %s\n",
			       fd, strerror(errno));
	}

	free(cfg);

#endif				/* !defined(SVEN_UNIT_TEST) */

	/** Initialize STH channel pointers inside the page
	*/
	for (i = 0; i < STH_CHANNELS_PER_PAGE; ++i) {
		block->channel[i] = sth_mmio++;
		DEBUG_OUT("    MMIO-SLOT %-2d = %p\n", i,
			  block->channel[i]);
	}
	return block;
}

/** Release a channel block back to the driver.
 * @param diag_on enable error message print out if != 0
 * @return pointer to new STH channel block or NULL on erro
 */
static void free_sth_block(psth_channel_block block, int diag_on)
{
#if defined(SVEN_UNIT_TEST)
	return;
#else

	DEBUG_OUT("free_sth_block(%p)\n", block);

	/* Unmap the STH channel memory from process.
	 */
	if (munmap((void *) block->channel[0], PAGE_SIZE)) {
		PRINT_ERROR_IF(diag_on,
			       "unable to unmap STH block : %s\n",
			       strerror(errno));
	}
#endif
}

/**
 * Platform specific SVEN handle initialization hook function
 *
 * @param svenh pointer to the new SVEN handle structure
 * @param platform_data user defined data for the init function.
 *
 * This function computes a STH channel MMIO pointer and stores it in the
 * handle data for direct memory data output.
 */
static void platform_handle_init(psven_handle_t svenh,
				 const void *platform_data __attribute__((unused)))
{
	psven_platform_state_t state;
	psth_channel_block block;
	int clz;

	state = &svenh->svh_header->svh_platform;
	svenh->svh_platform.sth_iomem = &state->sth_fake_io;

	pthread_mutex_lock(&state_mutex);

	/* Walk all page blocks to search for a free channel. A free
	 * channel is identified by a "1" bit in the block's free_bits
	 * mask. If there is at least one bit set in a block, we find it
	 * quickly using the intrinsic for "BSR" (__builtin_clzll) and
	 * then consume it by clearing this bit.
	 */
	for (block = state->page_block_chain; block != NULL;
	     block = block->next) {
		if (block->free_bits) {
			/* find and consume channel bit
			 */
			clz = 63 - __builtin_clzll(block->free_bits);
			block->free_bits &= ~(0x1ull << clz);

			/* Put channel MMIO pointer into the handle for direct
			 * memory access by the SVENTX output writer.
			 */
			svenh->svh_platform.sth_iomem =
			    block->channel[clz];

			/* Cache block/channel index in handle to speedup the
			 * later handle release processing.
			 */
			svenh->svh_platform.page_block = block;
			svenh->svh_platform.page_index = clz;

			DEBUG_OUT(
			  "platform_handle_init(%p) MMIO=%p,block=%p,slot=%d\n",
			  svenh, svenh->svh_platform.sth_iomem,
			  svenh->svh_platform.page_block,
			  svenh->svh_platform.page_index);
			break;
		}
	}

	if (svenh->svh_platform.sth_iomem == &state->sth_fake_io) {
		/* All used, need a new channel block from NPK driver
		 */
		block =
		    request_sth_block(state->diag_msg);

		if (NULL != block) {
			block->next = state->page_block_chain;
			if (NULL != block->next)
				block->next->prev = block;

			state->page_block_chain = block;

			clz = 63 - __builtin_clzll(block->free_bits);
			block->free_bits &= ~(0x1ull << clz);

			svenh->svh_platform.sth_iomem =
			    block->channel[clz];
			svenh->svh_platform.page_block = block;
			svenh->svh_platform.page_index = clz;

			DEBUG_OUT(
			  "platform_handle_init(%p) MMIO=%p,block=%p,slot=%d\n",
			  svenh, svenh->svh_platform.sth_iomem,
			  svenh->svh_platform.page_block,
			  svenh->svh_platform.page_index);
		}
	}

	pthread_mutex_unlock(&state_mutex);

#if defined(SVENTX_DEBUG)
	if (svenh->svh_platform.sth_iomem == &state->sth_fake_io)
		DEBUG_OUT("platform_handle_init(%p) with FAKE channel\n",
			  svenh);
#endif
}

/**
 * Platform specific SVEN handle initialization hook function
 *
 * @param svenh pointer to the new SVEN handle structure
 */
static void platform_handle_release(psven_handle_t svenh)
{
	psth_channel_block block;

	if (svenh->svh_platform.sth_iomem ==
	    &svenh->svh_header->svh_platform.sth_fake_io) {
		/* fake handle, no cleanup necessary */
		DEBUG_OUT("platform_handle_release(%p) with FAKE handle\n",
			  svenh);

		return;
	}

	DEBUG_OUT
	    ("platform_handle_release(%p) MMIO=%p, block=%p, slot=%d\n",
	     svenh, svenh->svh_platform.sth_iomem,
	     svenh->svh_platform.page_block,
	     svenh->svh_platform.page_index);

	block = (psth_channel_block) svenh->svh_platform.page_block;

	pthread_mutex_lock(&state_mutex);

	block->free_bits |= (0x1ull << svenh->svh_platform.page_index);

	if (block->free_bits == BLOCK_FREE_MASK) {
		/* All channels unused, unlink block from block chain for
		 * removal.
		 */
		if (block ==
		    svenh->svh_header->svh_platform.page_block_chain) {
			svenh->svh_header->svh_platform.page_block_chain =
			    block->next;
		}

		if (block->prev)
			block->prev->next = block->next;
		if (block->next)
			block->next->prev = block->prev;

		free_sth_block(block,
			       svenh->svh_header->svh_platform.diag_msg);
	}

	pthread_mutex_unlock(&state_mutex);
}

/**
 * Platform specific global SVEN state initialization hook function
 *
 * @param svenh pointer to the new SVEN handle structure
 * @param platform_data user defined data for the init function.
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_platform_init(psven_header_t svenh,
		     const void *platform_data __attribute__((unused)))
{
	/* Set handle init hook that performs per SVEN handle initialization
	 * and destruction
	 */
	svenh->svh_inith = platform_handle_init;
	svenh->svh_releaseh = platform_handle_release;

#if !defined(SVEN_UNIT_TEST)
#if defined(SVENTX_DEBUG)
	svenh->svh_platform.diag_msg = 1;
	fp_debug = fopen("sventx.log", "w");
#else
	/* Enable diagnostic messages to stderr if SVENTX_DEBUG is set
	 */
	{
		const char *p = getenv("SVENTX_DEBUG");
		if (NULL != p && strcmp(p, "0"))
			svenh->svh_platform.diag_msg = 1;
		else
			svenh->svh_platform.diag_msg = 0;
	}
#endif

	DEBUG_OUT("sventx_platform_init(%p, %p)\n", svenh, platform_data);
#endif
}

SVEN_EXPORT void SVEN_CALLCONV sventx_platform_destroy(psven_header_t
						       svenh __attribute__((unused)))
{
#if !defined(SVEN_UNIT_TEST)

	DEBUG_OUT("sventx_platform_destroy(%p)\n", svenh);

#if defined(SVENTX_DEBUG)
	if (NULL != fp_debug) {
		fclose(fp_debug);
		fp_debug = NULL;
	}
#endif
#endif

}

/**
 * User Mode shared library initialization /destruction
 */
static SVEN_SHAREDLIB_CONSTRUCTOR void shared_library_init()
{

	pthread_mutex_init(&state_mutex, NULL);

	/* Initialize SVEN infrastructure
	 * This must be done once at platform startup.
	 * The parameters are the platform specific initialization function and
	 * the data that gets passed to it.
	 */
	SVEN_INIT(sventx_platform_init, NULL);
}

/**
 * This example platform  uses SVENTX as a shared library inside an
 * application. The platform destroy hook is called during a shared library
 * destructor call.
 */
static SVEN_SHAREDLIB_DESTRUCTOR void shared_library_exit()
{
	/* run platform shutdown code */
	SVEN_SHUTDOWN(sventx_platform_destroy);

	pthread_mutex_destroy(&state_mutex);
}

sven_u64_t sventx_get_epoch_us()
{
	sven_u64_t epoch;
#if defined(SVEN_UNIT_TEST)
	epoch = 0x12345678aabbccddull;
#else
        struct timeval    tv;

        gettimeofday(&tv, NULL);
        epoch = tv.tv_sec;
	epoch *= 1000000;
	epoch += tv.tv_usec;
#endif
	return epoch;
}

#endif				/* ! __KERNEL__ */
