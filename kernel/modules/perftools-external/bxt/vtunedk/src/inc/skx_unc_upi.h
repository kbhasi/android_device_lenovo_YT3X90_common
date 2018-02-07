/*COPYRIGHT**
    Copyright (C) 2015 Intel Corporation.  All Rights Reserved.

    This file is part of SEP Development Kit

    SEP Development Kit is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    SEP Development Kit is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SEP Development Kit; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
**COPYRIGHT*/

#ifndef _SKYLAKE_SERVER_QPILL_H_INC_
#define _SKYLAKE_SERVER_QPILL_H_INC_

//Intel QPI Link 0	2F80h, 2F32h, 2F83h	8	0,2,3	Intel QPI Link 0
//Intel QPI Link 1	2F90h, 2F33h, 2F93h	9	0,2,3	Intel QPI Link 1
//Intel QPI Link 2	2F40h, 2F3Ah, 2F43h	10	0,2,3	Intel QPI Link 2
#define SKYLAKE_SERVER_QPILL0_DID              0x2058 // --- QPILL0 PerfMon DID --- B:D :14:0
#define SKYLAKE_SERVER_QPILL1_DID              0x2058 // --- QPILL1 PerfMon DID --- B:D :15:0
#define SKYLAKE_SERVER_QPILL2_DID              0x2058 // --- QPILL2 PerfMon DID --- B:D :16:0 -- EX
#define SKYLAKE_SERVER_QPILL_MM0_DID           0x2058 // --- QPILL0 PerfMon MM Config DID --- B:D :14:0
#define SKYLAKE_SERVER_QPILL_MM1_DID           0x2058 // --- QPILL1 PerfMon MM Config DID --- B:D :15:0
#define SKYLAKE_SERVER_QPILL_MM2_DID           0x2058 // --- QPILL1 PerfMon MM Config DID --- B:D :16:0 -- EX

#define NEXT_ADDR_OFFSET                          4
#define QPILL_ADDR_SHIFT                          32
#define DRV_IS_PCI_VENDOR_ID_INTEL                0x8086

#define SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR              0x700

#define ENABLE_SKYLAKE_SERVER_QPI_COUNTERS                       0x400000
#define DISABLE_SKYLAKE_SERVER_QPI_COUNTERS                      0x100
#define ENABLE_ALL_SKYLAKE_SERVER_PMU                            0x2000000000000000ULL
#define DISABLE_ALL_SKYLAKE_SERVER_PMU                           0x8000000000000000ULL
#define SKYLAKE_SERVER_QPILL_UNIT_COUNTER_UNIT_CONTROL_OFFSET    0x78
#define SKYLAKE_SERVER_QPILL_UNIT_COUNTER_0_CONTROL_OFFSET       0x350
#define SKYLAKE_SERVER_QPILL_UNIT_COUNTER_1_CONTROL_OFFSET       0x358
#define SKYLAKE_SERVER_QPILL_UNIT_COUNTER_2_CONTROL_OFFSET       0x360
#define SKYLAKE_SERVER_QPILL_UNIT_COUNTER_3_CONTROL_OFFSET       0x368
#define IS_THIS_SKYLAKE_SERVER_QPILL_PCI_UNIT_CTL(x)             (x == SKYLAKE_SERVER_QPILL_UNIT_COUNTER_UNIT_CONTROL_OFFSET)
#define IS_THIS_SKYLAKE_SERVER_QPILL_PCI_PMON_CTL(x)             ((x == SKYLAKE_SERVER_QPILL_UNIT_COUNTER_0_CONTROL_OFFSET) || \
                                                                  (x == SKYLAKE_SERVER_QPILL_UNIT_COUNTER_1_CONTROL_OFFSET) || \
                                                                  (x == SKYLAKE_SERVER_QPILL_UNIT_COUNTER_2_CONTROL_OFFSET) || \
                                                                  (x == SKYLAKE_SERVER_QPILL_UNIT_COUNTER_3_CONTROL_OFFSET))

#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK0_OFFSET                 0x258
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK1_OFFSET                 0x25c
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK2_OFFSET                 0x260
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK3_OFFSET                 0x264
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK4_OFFSET                 0x268
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK5_OFFSET                 0x26C
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MASK6_OFFSET                 0x270

#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH0_OFFSET                0x23C
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH1_OFFSET                0x240
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH2_OFFSET                0x244
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH3_OFFSET                0x248
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH4_OFFSET                0x2FC
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH5_OFFSET                0x250
#define SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH6_OFFSET                0x254

#define IS_THIS_SKYLAKE_SERVER_QPILL_MASK_REG(x)                  ((x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK0_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK1_OFFSET)  || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK2_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK3_OFFSET)  || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK4_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK5_OFFSET)  || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MASK6_OFFSET))
#define IS_THIS_SKYLAKE_SERVER_QPILL_MATCH_REG(x)                 ((x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH0_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH1_OFFSET) || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH2_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH3_OFFSET) || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH4_OFFSET) || (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH5_OFFSET) || \
                                                                   (x == SKYLAKE_SERVER_QPILL_UNIT_RX_MATCH6_OFFSET))
#define IS_THIS_SKYLAKE_SERVER_QPILL_MM_REG(x)                    (IS_THIS_SKYLAKE_SERVER_QPILL_MASK_REG(x) || IS_THIS_SKYLAKE_SERVER_QPILL_MATCH_REG(x))

extern  DISPATCH_NODE                                            skx_qpill_dispatch;
#define RESET_QPI_CTRS                                           0x2

#endif
