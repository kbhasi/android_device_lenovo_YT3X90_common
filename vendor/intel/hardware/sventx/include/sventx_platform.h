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

/* Example platform specific extensions
 * This "platform" shows how to use the SVEN platform modules
 * to configure the SVEN feature set and add platform specific
 * customizations.
 */

#ifndef SVEN_PLATFORM_INCLUDED
#define SVEN_PLATFORM_INCLUDED

#include "sventx_platform_config.h"

#if defined(SVEN_PCFG_ENABLE_HEAP_MEMORY)
/**
 * Map malloc style heap allocation function to SVENTX macro.
 * This function is used for handle allocations if heap usage
 * is supported by the platform.
 */

#if defined(__KERNEL__)

#include <linux/slab.h>
#include <linux/percpu.h>

#define SVEN_HEAP_MALLOC(s) kmalloc((s), GFP_KERNEL)
#define SVEN_HEAP_FREE(p)   kfree(p)

#else
#include <stdlib.h>

#define SVEN_HEAP_MALLOC(s) malloc(s)
#define SVEN_HEAP_FREE(p)   free(p)
#endif

#endif		/* defined(SVEN_PCFG_ENABLE_HEAP_MEMORY) */


#ifdef __cplusplus
extern "C" {
#endif


/*
 * Data input register on STH (Dn, DnM or DnMTS)
 */
union sven_dn_sth_reg {
	sven_u8_t d8;
	sven_u16_t d16;
	sven_u32_t d32;
	sven_u64_t d64;
} __attribute__ ((__packed__));

/*
 * NPK STH channel memory layout
 */
struct sven_sth_channel {
	union sven_dn_sth_reg dn;
	union sven_dn_sth_reg dnm;
	union sven_dn_sth_reg dnts;
	union sven_dn_sth_reg dnmts;
	union sven_dn_sth_reg user;
	union sven_dn_sth_reg userts;
	sven_u32_t flag;
	sven_u32_t flagts;
	sven_u32_t merr;
	sven_u32_t reserved;

} __attribute__ ((__packed__));


struct s_sth_channel_block;

/**
 * Platform specific SVEN global state extension
 *
 * The contents of this structure can be freely defined to
 * match platform specific data needs. It can later be
 * accessed through the sven_header svh_platform member.
 */
typedef struct s_sven_platform_state {
	int npkdrv_errno;
		    /**< errno value if device access failed  */
	int diag_msg;
		    /**< print diagnostic messages if set     */

	/**
	* STH fake memory location that is used for IO if no STH
	* channels are available. This frees the trace sending code
	* from testing the pointer, also all trace output will get lost.
	*/
	struct sven_sth_channel sth_fake_io;

	/**
	 * NPK Driver channel blocks
	 */
	struct s_sth_channel_block *page_block_chain;
} sven_platform_state_t, *psven_platform_state_t;


extern SVEN_EXPORT void SVEN_CALLCONV
sventx_platform_init(struct s_sven_header *, const void *);
extern SVEN_EXPORT void SVEN_CALLCONV
sventx_platform_destroy(struct s_sven_header *svenh);

#if defined(SVEN_PCFG_ENABLE_CLOCK_API)
extern sven_u64_t sventx_get_epoch_us(void);
#define SVEN_PLATFORM_CLOCK sventx_get_epoch_us
#endif /* defined(SVEN_PCFG_ENABLE_CLOCK_API) */

#if !defined(__KERNEL__)

/**
 * Platform specific SVEN handle state extension
 *
 * The contents of this structure can be freely defined to
 * match platform specific data needs. It can later be
 * accessed through the sven_handles's svh_platform member.
 *
 * @see SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA sven_handle_t
 */
typedef struct s_sven_platform_handle {
	/** pointer to NPK STH channel mmio area */
	volatile struct sven_sth_channel *sth_iomem;

	/** page block owning sth_iomem */
	const void *page_block;
	/** pointer index in page */
	int page_index;

} sven_platform_handle_t, *psven_platform_handle_t;

#endif


/* STH IO output routine mapping */

#if defined(__KERNEL__)

#define SVENTX_PERCPU_CHANNELS 4

DECLARE_PER_CPU(volatile struct sven_sth_channel *, sventx_sth_mmio);
DECLARE_PER_CPU(int, sventx_cpu_nesting_level);
extern struct sven_sth_channel *
	sventx_sth_channels[NR_CPUS][SVENTX_PERCPU_CHANNELS+1];
extern int sventx_debug_level;

/* SVENTX Error codes
 * These Codes are used as upper 16bit of a 32bit error value. The lower
 * 16 bits are an (optional) parameter for the code.
 */
#define ERR_CPU_OVERFL 0xE000

#define SVENTX_MK_ECODE(c, p) ((sven_u32_t)((c << 16) | (p & 0xFFFF)))


/* Helper to emit messages based on debug level module parameter */
#define debug_print(level, ...) \
	{ if (sventx_debug_level >= level) pr_info(__VA_ARGS__); }

#if 0
#define DEBUG_STH_IO(s, m, d) \
	do { volatile struct sven_sth_channel *p; \
		p = per_cpu(sventx_sth_mmio, smp_processor_id());\
		pr_info("SVENTX: STH-IO %7s on CPU %d:%d @ %p = %llx\n",\
			s, smp_processor_id(),\
			per_cpu(sventx_cpu_nesting_level, smp_processor_id()), \
			&p->m, (unsigned long long)d); \
	} while (0)
#else
#define DEBUG_STH_IO(s, m, d)
#endif

static inline void sventx_aquire_percpu_sth_ptr(void)
{
	/* Get next free channel on current CPU stack and prevent preemption
	 */
	int level = ++get_cpu_var(sventx_cpu_nesting_level);
	int cpu   = smp_processor_id();

	if (level < SVENTX_PERCPU_CHANNELS) {
		per_cpu(sventx_sth_mmio, cpu) =
			sventx_sth_channels[cpu][level];
	} else {
		/* Nesting level got too deep to handle for this cpu. Redirect
		 * trace into dummy region (nesting level 0). This trace record
		 * will be lost.
		 */
		per_cpu(sventx_sth_mmio, cpu) =
			sventx_sth_channels[cpu][0];

		/* Send overflow error code as atomic D32MTS on reserved error
		 * channel to reliable indicate this problem in the trace.
		 * Don't report nesting larger then 10 as the decode will show
		 * ecode only up to 10, with 10 shown as >= 10.
		 */
		level = (level > 10) ? 10 : level;
		sventx_sth_channels[cpu][SVENTX_PERCPU_CHANNELS]->dnmts.d32 =
			SVENTX_MK_ECODE(ERR_CPU_OVERFL, level);
	}
}

static inline void sventx_release_percpu_sth_ptr(void)
{
	int cpu   = smp_processor_id();
	int level = --per_cpu(sventx_cpu_nesting_level, cpu);

	if (level < SVENTX_PERCPU_CHANNELS)
		per_cpu(sventx_sth_mmio, cpu) =
			sventx_sth_channels[cpu][level];

	/* IO done, reenable preemption.
	 */
	put_cpu_var(sventx_cpu_nesting_level);
}

#define get_cpu_mmio_ptr() \
	per_cpu(sventx_sth_mmio, smp_processor_id())

/*
 * Use STH channel MMIO pointer per_cpu variable to access the NPK.
 * Each record starts with a D32TS and ends with a FLAG. The code
 * acquires the per_pcu STH MMIO pointer on the D32TS, which internally
 * suppresses preemption from this CPU, resulting in an atomic record
 * output operation on this channel. Preemption resumes after
 * issuing a FLAG on the STH (put_cpu_var()).
 */
#define SVEN_STH_OUT_D32TS(sven_handle, data) \
	{\
		sventx_aquire_percpu_sth_ptr();\
		get_cpu_mmio_ptr()->dnts.d32 = data;\
		DEBUG_STH_IO("D32TS", dnts.d32, data);\
	}

#define SVEN_STH_OUT_D8(sven_handle, data) \
	{\
		get_cpu_mmio_ptr()->dn.d8 = data;\
		DEBUG_STH_IO("D8", dn.d8, data);\
	}

#define SVEN_STH_OUT_D16(sven_handle, data) \
	{\
		get_cpu_mmio_ptr()->dn.d16 = data;\
		DEBUG_STH_IO("D16", dn.d16, data);\
	}

#define SVEN_STH_OUT_D32(sven_handle, data) \
	{\
		get_cpu_mmio_ptr()->dn.d32 = data;\
		DEBUG_STH_IO("D32", dn.d32, data);\
	}

#define SVEN_STH_OUT_D64(sven_handle, data) \
	{\
		get_cpu_mmio_ptr()->dn.d64 = data;\
		DEBUG_STH_IO("D64", dn.d64, data);\
	}

#define SVEN_STH_OUT_FLAG(sven_handle) \
	{\
		get_cpu_mmio_ptr()->flag = 0;\
		DEBUG_STH_IO("FLAG", flag, 0);\
		sventx_release_percpu_sth_ptr();\
	}

#define SVEN_STH_OUT_D32MTS(sven_handle, data) \
	{\
		sventx_aquire_percpu_sth_ptr(); \
		get_cpu_mmio_ptr()->dnmts.d32 = data; \
		DEBUG_STH_IO("D32MTS", dnmts.d32, data);\
		sventx_release_percpu_sth_ptr(); \
	}

#define SVEN_STH_OUT_USER8TS(sven_handle, data) \
	{\
		sventx_aquire_percpu_sth_ptr(); \
		get_cpu_mmio_ptr()->userts.d8 = data;\
		DEBUG_STH_IO("USERTS", userts.d8, data)\
		sventx_release_percpu_sth_ptr();\
	}

inline sven_u64_t sventx_get_epoch_us()
{
	sven_u64_t epoch;
	struct timespec ts;

	getnstimeofday(&ts);

	epoch  = ts.tv_sec  * 1000000;
	epoch += ts.tv_nsec / 1000;
	return epoch;
}

#else		/* !defined(__KERNEL__) */

#define SVEN_STH_OUT_D32TS(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dnts.d32 = data; }

#define SVEN_STH_OUT_D8(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dn.d8 = data; }

#define SVEN_STH_OUT_D16(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dn.d16 = data; }

#define SVEN_STH_OUT_D32(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dn.d32 = data; }

#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
#define SVEN_STH_OUT_D64(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dn.d64 = data; }
#endif

#define SVEN_STH_OUT_FLAG(svh) \
	{ (svh)->svh_platform.sth_iomem->flag = 0; }

#define SVEN_STH_OUT_D32MTS(svh, data) \
	{ (svh)->svh_platform.sth_iomem->dnmts.d32 = data; }

#define SVEN_STH_OUT_USER8TS(svh, data) \
	{ (svh)->svh_platform.sth_iomem->userts.d8 = data; }

#endif

#if defined(SVEN_UNIT_TEST)
#define SVEN_UNIT_TEST_LINUX_PLATFORM
#endif

#ifdef __cplusplus
}		/* extern C */
#endif
#endif
