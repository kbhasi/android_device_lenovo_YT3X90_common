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

#ifndef _SKYLAKE_SERVER_IMC_H_INC_
#define _SKYLAKE_SERVER_IMC_H_INC_

// Channel 0
#define SKYLAKE_SERVER_IMC0_DID             0x2042 // --- IMC0 DID -- 2:10:2
#define SKYLAKE_SERVER_IMC1_DID             0x2046 // --- IMC1 DID -- 2:10:6
#define SKYLAKE_SERVER_IMC2_DID             0x204A // --- IMC2 DID -- 2:11:2
// Channel 1
#define SKYLAKE_SERVER_IMC3_DID             0x2042 // --- IMC0 DID -- 2:12:2
#define SKYLAKE_SERVER_IMC4_DID             0x2046 // --- IMC1 DID -- 2:12:6
#define SKYLAKE_SERVER_IMC5_DID             0x204A // --- IMC2 DID -- 2:13:2

#define SKYLAKE_SERVER_IMC_UNIT_COUNTER_UNIT_CONTROL_OFFSET    0xf4  
#define SKYLAKE_SERVER_IMC_UNIT_COUNTER_0_CONTROL_OFFSET       0xd8
#define SKYLAKE_SERVER_IMC_UNIT_COUNTER_1_CONTROL_OFFSET       0xdc
#define SKYLAKE_SERVER_IMC_UNIT_COUNTER_2_CONTROL_OFFSET       0xe0
#define SKYLAKE_SERVER_IMC_UNIT_COUNTER_3_CONTROL_OFFSET       0xe4
#define SKYLAKE_SERVER_IMC_UNIT_FIXED_COUNTER_CONTROL_OFFSET   0xf0

#define NEXT_ADDR_OFFSET                         4
#define IMC_ADDR_SHIFT                           32
#define DRV_IS_PCI_VENDOR_ID_INTEL               0x8086
#define ENABLE_SKYLAKE_SERVER_IMC_COUNTERS       0x400000
#define DISABLE_SKYLAKE_SERVER_IMC_COUNTERS      0x100

#define IS_THIS_SKYLAKE_SERVER_BOX_MC_PCI_UNIT_CTL(x)     (x == SKYLAKE_SERVER_IMC_UNIT_COUNTER_UNIT_CONTROL_OFFSET) 
#define IS_THIS_SKYLAKE_SERVER_MC_PCI_PMON_CTL(x)         (  (x == SKYLAKE_SERVER_IMC_UNIT_COUNTER_0_CONTROL_OFFSET)  \
                                                          || (x == SKYLAKE_SERVER_IMC_UNIT_COUNTER_1_CONTROL_OFFSET)  \
                                                          || (x == SKYLAKE_SERVER_IMC_UNIT_COUNTER_2_CONTROL_OFFSET)  \
                                                          || (x == SKYLAKE_SERVER_IMC_UNIT_COUNTER_3_CONTROL_OFFSET)  \
                                                          || (x == SKYLAKE_SERVER_IMC_UNIT_FIXED_COUNTER_CONTROL_OFFSET))
extern DISPATCH_NODE  skx_imc_dispatch;
#define RESET_IMC_CTRS                                    0x10002

#endif
