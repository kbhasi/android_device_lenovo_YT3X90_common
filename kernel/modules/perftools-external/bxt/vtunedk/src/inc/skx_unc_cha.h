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

#ifndef _SKX_CHA_H_INC_
#define _SKX_CHA_H_INC_


#define SKX_CHA0_BOX_CTL_MSR                     0xE00
#define SKX_CHA1_BOX_CTL_MSR                     0xE10
#define SKX_CHA2_BOX_CTL_MSR                     0xE20
#define SKX_CHA3_BOX_CTL_MSR                     0xE30
#define SKX_CHA4_BOX_CTL_MSR                     0xE40
#define SKX_CHA5_BOX_CTL_MSR                     0xE50
#define SKX_CHA6_BOX_CTL_MSR                     0xE60
#define SKX_CHA7_BOX_CTL_MSR                     0xE70
#define SKX_CHA8_BOX_CTL_MSR                     0xE80
#define SKX_CHA9_BOX_CTL_MSR                     0xE90
#define SKX_CHA10_BOX_CTL_MSR                    0xEA0
#define SKX_CHA11_BOX_CTL_MSR                    0xEB0
#define SKX_CHA12_BOX_CTL_MSR                    0xEC0
#define SKX_CHA13_BOX_CTL_MSR                    0xED0
#define SKX_CHA14_BOX_CTL_MSR                    0xEE0
#define SKX_CHA15_BOX_CTL_MSR                    0xEF0
#define SKX_CHA16_BOX_CTL_MSR                    0xF00
#define SKX_CHA17_BOX_CTL_MSR                    0xF10
#define SKX_CHA18_BOX_CTL_MSR                    0xF20
#define SKX_CHA19_BOX_CTL_MSR                    0xF30
#define SKX_CHA20_BOX_CTL_MSR                    0xF40
#define SKX_CHA21_BOX_CTL_MSR                    0xF50
#define SKX_CHA22_BOX_CTL_MSR                    0xF60
#define SKX_CHA23_BOX_CTL_MSR                    0xF70
#define SKX_CHA24_BOX_CTL_MSR                    0xF80
#define SKX_CHA25_BOX_CTL_MSR                    0xF90
#define SKX_CHA26_BOX_CTL_MSR                    0xFA0
#define SKX_CHA27_BOX_CTL_MSR                    0xFB0

#define SKX_CHA27_PMON_CTL0_MSR                  0xFB1
#define SKX_CHA27_PMON_CTL3_MSR                  0xFB4
#define SKX_CHA26_PMON_CTL0_MSR                  0xFA1
#define SKX_CHA26_PMON_CTL3_MSR                  0xFA4
#define SKX_CHA25_PMON_CTL0_MSR                  0xF91
#define SKX_CHA25_PMON_CTL3_MSR                  0xF94
#define SKX_CHA24_PMON_CTL0_MSR                  0xF81
#define SKX_CHA24_PMON_CTL3_MSR                  0xF84
#define SKX_CHA23_PMON_CTL0_MSR                  0xF71
#define SKX_CHA23_PMON_CTL3_MSR                  0xF74
#define SKX_CHA22_PMON_CTL0_MSR                  0xF61
#define SKX_CHA22_PMON_CTL3_MSR                  0xF64
#define SKX_CHA21_PMON_CTL0_MSR                  0xF51
#define SKX_CHA21_PMON_CTL3_MSR                  0xF54
#define SKX_CHA20_PMON_CTL0_MSR                  0xF41
#define SKX_CHA20_PMON_CTL3_MSR                  0xF44
#define SKX_CHA19_PMON_CTL0_MSR                  0xF31
#define SKX_CHA19_PMON_CTL3_MSR                  0xF34
#define SKX_CHA18_PMON_CTL0_MSR                  0xF21
#define SKX_CHA18_PMON_CTL3_MSR                  0xF24
#define SKX_CHA17_PMON_CTL0_MSR                  0xF11
#define SKX_CHA17_PMON_CTL3_MSR                  0xF14
#define SKX_CHA16_PMON_CTL0_MSR                  0xF01
#define SKX_CHA16_PMON_CTL3_MSR                  0xF04
#define SKX_CHA15_PMON_CTL0_MSR                  0xEF1
#define SKX_CHA15_PMON_CTL3_MSR                  0xEF4
#define SKX_CHA14_PMON_CTL0_MSR                  0xEE1
#define SKX_CHA14_PMON_CTL3_MSR                  0xEE4
#define SKX_CHA13_PMON_CTL0_MSR                  0xED1
#define SKX_CHA13_PMON_CTL3_MSR                  0xED4
#define SKX_CHA12_PMON_CTL0_MSR                  0xEC1
#define SKX_CHA12_PMON_CTL3_MSR                  0xEC4
#define SKX_CHA11_PMON_CTL0_MSR                  0xEB1
#define SKX_CHA11_PMON_CTL3_MSR                  0xEB4
#define SKX_CHA10_PMON_CTL0_MSR                  0xEA1
#define SKX_CHA10_PMON_CTL3_MSR                  0xEA4
#define SKX_CHA9_PMON_CTL0_MSR                   0xE91
#define SKX_CHA9_PMON_CTL3_MSR                   0xE94
#define SKX_CHA8_PMON_CTL0_MSR                   0xE81
#define SKX_CHA8_PMON_CTL3_MSR                   0xE84
#define SKX_CHA7_PMON_CTL0_MSR                   0xE71
#define SKX_CHA7_PMON_CTL3_MSR                   0xE74
#define SKX_CHA6_PMON_CTL0_MSR                   0xE61
#define SKX_CHA6_PMON_CTL3_MSR                   0xE64
#define SKX_CHA5_PMON_CTL0_MSR                   0xE51
#define SKX_CHA5_PMON_CTL3_MSR                   0xE54
#define SKX_CHA4_PMON_CTL0_MSR                   0xE41
#define SKX_CHA4_PMON_CTL3_MSR                   0xE44
#define SKX_CHA3_PMON_CTL0_MSR                   0xE31
#define SKX_CHA3_PMON_CTL3_MSR                   0xE34
#define SKX_CHA2_PMON_CTL0_MSR                   0xE21
#define SKX_CHA2_PMON_CTL3_MSR                   0xE24
#define SKX_CHA1_PMON_CTL0_MSR                   0xE11
#define SKX_CHA1_PMON_CTL3_MSR                   0xE14
#define SKX_CHA0_PMON_CTL0_MSR                   0xE01
#define SKX_CHA0_PMON_CTL3_MSR                   0xE04

#define IS_THIS_SKX_BOX_CTL_MSR(x)              ( x == SKX_CHA0_BOX_CTL_MSR  || x == SKX_CHA1_BOX_CTL_MSR  || x == SKX_CHA2_BOX_CTL_MSR  || x == SKX_CHA3_BOX_CTL_MSR \
                                               || x == SKX_CHA4_BOX_CTL_MSR  || x == SKX_CHA5_BOX_CTL_MSR  || x == SKX_CHA6_BOX_CTL_MSR  || x == SKX_CHA7_BOX_CTL_MSR \
                                               || x == SKX_CHA8_BOX_CTL_MSR  || x == SKX_CHA9_BOX_CTL_MSR  || x == SKX_CHA10_BOX_CTL_MSR || x == SKX_CHA11_BOX_CTL_MSR \
                                               || x == SKX_CHA12_BOX_CTL_MSR || x == SKX_CHA13_BOX_CTL_MSR || x == SKX_CHA14_BOX_CTL_MSR || x == SKX_CHA15_BOX_CTL_MSR \
                                               || x == SKX_CHA16_BOX_CTL_MSR || x == SKX_CHA17_BOX_CTL_MSR || x == SKX_CHA18_BOX_CTL_MSR || x == SKX_CHA19_BOX_CTL_MSR \
                                               || x == SKX_CHA20_BOX_CTL_MSR || x == SKX_CHA21_BOX_CTL_MSR || x == SKX_CHA22_BOX_CTL_MSR || x == SKX_CHA23_BOX_CTL_MSR \
                                               || x == SKX_CHA24_BOX_CTL_MSR || x == SKX_CHA25_BOX_CTL_MSR || x == SKX_CHA26_BOX_CTL_MSR || x == SKX_CHA27_BOX_CTL_MSR)

#define  DISABLE_SKX_CHA_COUNTERS                0x100
#define  ENABLE_SKX_CHA_COUNTERS                 0x400000

#define IS_THIS_SKX_EVSEL_PMON_CTL_MSR(x)              ( (x>= SKX_CHA0_PMON_CTL0_MSR  && x <= SKX_CHA0_PMON_CTL3_MSR)  || (x>= SKX_CHA1_PMON_CTL0_MSR && x <= SKX_CHA1_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA2_PMON_CTL0_MSR  && x <= SKX_CHA2_PMON_CTL3_MSR)  || (x>= SKX_CHA3_PMON_CTL0_MSR && x <= SKX_CHA3_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA4_PMON_CTL0_MSR  && x <= SKX_CHA4_PMON_CTL3_MSR)  || (x>= SKX_CHA5_PMON_CTL0_MSR && x <= SKX_CHA5_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA6_PMON_CTL0_MSR  && x <= SKX_CHA6_PMON_CTL3_MSR)  || (x>= SKX_CHA7_PMON_CTL0_MSR && x <= SKX_CHA7_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA8_PMON_CTL0_MSR  && x <= SKX_CHA8_PMON_CTL3_MSR)  || (x>= SKX_CHA9_PMON_CTL0_MSR && x <= SKX_CHA9_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA10_PMON_CTL0_MSR && x <= SKX_CHA10_PMON_CTL3_MSR) || (x>= SKX_CHA11_PMON_CTL0_MSR && x <= SKX_CHA11_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA12_PMON_CTL0_MSR && x <= SKX_CHA12_PMON_CTL3_MSR) || (x>= SKX_CHA13_PMON_CTL0_MSR && x <= SKX_CHA13_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA14_PMON_CTL0_MSR && x <= SKX_CHA14_PMON_CTL3_MSR) || (x>= SKX_CHA15_PMON_CTL0_MSR && x <= SKX_CHA15_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA16_PMON_CTL0_MSR && x <= SKX_CHA16_PMON_CTL3_MSR) || (x>= SKX_CHA17_PMON_CTL0_MSR && x <= SKX_CHA17_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA18_PMON_CTL0_MSR && x <= SKX_CHA18_PMON_CTL3_MSR) || (x>= SKX_CHA19_PMON_CTL0_MSR && x <= SKX_CHA19_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA20_PMON_CTL0_MSR && x <= SKX_CHA20_PMON_CTL3_MSR) || (x>= SKX_CHA21_PMON_CTL0_MSR && x <= SKX_CHA21_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA22_PMON_CTL0_MSR && x <= SKX_CHA22_PMON_CTL3_MSR) || (x>= SKX_CHA23_PMON_CTL0_MSR && x <= SKX_CHA23_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA24_PMON_CTL0_MSR && x <= SKX_CHA24_PMON_CTL3_MSR) || (x>= SKX_CHA25_PMON_CTL0_MSR && x <= SKX_CHA25_PMON_CTL3_MSR) \
                                                      || (x>= SKX_CHA26_PMON_CTL0_MSR && x <= SKX_CHA26_PMON_CTL3_MSR) || (x>= SKX_CHA27_PMON_CTL0_MSR && x <= SKX_CHA27_PMON_CTL3_MSR) )

extern  DISPATCH_NODE                            skx_cha_dispatch;
#define SKX_UBOX_GLOBAL_CONTROL_MSR              0x700

#endif

