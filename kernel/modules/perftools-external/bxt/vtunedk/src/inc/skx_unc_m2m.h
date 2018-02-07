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

#ifndef _SKYLAKE_SERVER_M2M_H_INC_
#define _SKYLAKE_SERVER_M2M_H_INC_

#define SKYLAKE_SERVER_M2M_DID                            0x2066

#define SKYLAKE_SERVER_M2M_COUNTER_UNIT_CONTROL_OFFSET    0x258
#define SKYLAKE_SERVER_M2M_COUNTER_0_CONTROL_OFFSET       0x228
#define SKYLAKE_SERVER_M2M_COUNTER_1_CONTROL_OFFSET       0x230
#define SKYLAKE_SERVER_M2M_COUNTER_2_CONTROL_OFFSET       0x238
#define SKYLAKE_SERVER_M2M_COUNTER_3_CONTROL_OFFSET       0x240


#define IS_THIS_SKYLAKE_SERVER_M2M_UNIT_CTL(x)            ( (x) ==  SKYLAKE_SERVER_M2M_COUNTER_UNIT_CONTROL_OFFSET)
#define IS_THIS_SKYLAKE_SERVER_M2M_PMON_CTL(x)            (   ((x) ==  SKYLAKE_SERVER_M2M_COUNTER_0_CONTROL_OFFSET) \
                                                           || ((x) ==  SKYLAKE_SERVER_M2M_COUNTER_1_CONTROL_OFFSET) \
                                                           || ((x) ==  SKYLAKE_SERVER_M2M_COUNTER_2_CONTROL_OFFSET) \
                                                           || ((x) ==  SKYLAKE_SERVER_M2M_COUNTER_3_CONTROL_OFFSET))

#define  DISABLE_SKYLAKE_SERVER_M2M_COUNTERS     0x100
#define  ENABLE_SKYLAKE_SERVER_M2M_COUNTERS      0x400000
#define  ENABLE_ALL_SKYLAKE_SERVER_M2M_PMU       0x2000000000000000ULL
#define  DISABLE_ALL_SKYLAKE_SERVER_M2M_PMU      0x8000000000000000ULL


extern  DISPATCH_NODE                                  skx_m2m_dispatch;
#define SKYLAKE_SERVER_UBOX_GLOBAL_CONTROL_MSR         0x700
#define RESET_M2M_CTRS                                  0x2
#endif
