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

#ifndef _SKX_IIO_H_INC_
#define _SKX_IIO_H_INC_


#define SKX_IIO0_BOX_CTL_MSR         0xA40
#define SKX_IIO1_BOX_CTL_MSR         0xA60
#define SKX_IIO2_BOX_CTL_MSR         0xA80
#define SKX_IIO3_BOX_CTL_MSR         0xAA0
#define SKX_IIO4_BOX_CTL_MSR         0xAC0

#define SKX_IIO4_PMON_CTL0_MSR       0xAC8
#define SKX_IIO4_PMON_CTL3_MSR       0xACB
#define SKX_IIO3_PMON_CTL0_MSR       0xAA8
#define SKX_IIO3_PMON_CTL3_MSR       0xAAB
#define SKX_IIO2_PMON_CTL0_MSR       0xA88
#define SKX_IIO2_PMON_CTL3_MSR       0xA8B
#define SKX_IIO1_PMON_CTL0_MSR       0xA68
#define SKX_IIO1_PMON_CTL3_MSR       0xA6B
#define SKX_IIO0_PMON_CTL0_MSR       0xA48
#define SKX_IIO0_PMON_CTL3_MSR       0xA4B

#define IS_THIS_SKX_IIO_BOX_CTL_MSR(x)        (   (x) == SKX_IIO0_BOX_CTL_MSR  \
                                               || (x) == SKX_IIO1_BOX_CTL_MSR  \
                                               || (x) == SKX_IIO2_BOX_CTL_MSR  \
                                               || (x) == SKX_IIO3_BOX_CTL_MSR  \
                                               || (x) == SKX_IIO4_BOX_CTL_MSR)

#define  DISABLE_SKX_IIO_COUNTERS     0x100
#define  ENABLE_SKX_IIO_COUNTERS      0x400000
#define  ENABLE_ALL_SKX_IIO_PMU       0x20000000
#define  DISABLE_ALL_SKX_IIO_PMU      0x80000000

#define IS_THIS_SKX_IIO_EVSEL_PMON_CTL_MSR(x)   (   ((x) >= SKX_IIO0_PMON_CTL0_MSR  && (x) <= SKX_IIO0_PMON_CTL3_MSR)  \
                                                    || ((x) >= SKX_IIO1_PMON_CTL0_MSR &&  (x) <= SKX_IIO1_PMON_CTL3_MSR) \
                                                    || ((x) >= SKX_IIO2_PMON_CTL0_MSR  && (x) <= SKX_IIO2_PMON_CTL3_MSR) \
                                                    || ((x) >= SKX_IIO3_PMON_CTL0_MSR  && (x) <= SKX_IIO3_PMON_CTL3_MSR) \
                                                    || ((x) >= SKX_IIO4_PMON_CTL0_MSR  && (x) <= SKX_IIO4_PMON_CTL3_MSR) )

extern  DISPATCH_NODE                       skx_iio_dispatch;
#define SKX_UBOX_GLOBAL_CONTROL_MSR         0x700
#define RESET_IIO_CTRS                      0x2
#endif
