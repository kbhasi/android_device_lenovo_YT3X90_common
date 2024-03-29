/*
    Copyright (C) 2011-2015 Intel Corporation.  All Rights Reserved.

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
*/

#ifndef _SKLUNC_CBO_H_INC_
#define _SKLUNC_CBO_H_INC_

/*
 * Local to this architecture: SKL uncore CBO unit 
 * Arch Perf monitoring version 4
 */
// CBO MSRs
#define SKL_CBO_PERF_GLOBAL_CTRL        0xE01
#define SKL_CBO_PERF_GLOBAL_STATUS      0xE02

extern DISPATCH_NODE  skylakeunc_cbo_dispatch;

#endif
