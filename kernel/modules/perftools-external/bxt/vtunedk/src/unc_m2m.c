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


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "ecb_iterators.h"
#include "hsx_unc_ubox.h"
#include "skx_unc_m2m.h"
#include "unc_common.h"



static DRV_BOOL
unc_m2m_skx_is_Valid (
    U32 device_id
)
{
    if (device_id != SKYLAKE_SERVER_M2M_DID) {
       return FALSE;
    }
    return TRUE;
}


static DRV_BOOL
unc_m2m_skx_is_Valid_For_Write (
    U32 device_id,
    U32 reg_id
)
{
    if (device_id != SKYLAKE_SERVER_M2M_DID) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_m2m_skl_is_Unit_Ctl (
    U32 msr_addr
)
{
    return (IS_THIS_SKYLAKE_SERVER_M2M_UNIT_CTL(msr_addr));
}

static DRV_BOOL
unc_m2m_skl_is_PMON_Ctl (
    U32 msr_addr
)
{
    return (IS_THIS_SKYLAKE_SERVER_M2M_PMON_CTL(msr_addr));
}

DEVICE_CALLBACK_NODE  unc_m2m_skl_callback = {
    unc_m2m_skx_is_Valid,
    unc_m2m_skx_is_Valid_For_Write,
    unc_m2m_skl_is_Unit_Ctl,
    unc_m2m_skl_is_PMON_Ctl
};

/******************************************************************************************
 * @fn          static VOID unc_m2m_skl_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 ******************************************************************************************/
static VOID
unc_m2m_skl_Write_PMU (
    VOID  *param
)
{
    UNC_COMMON_PCI_Write_PMU(param,
                             SKYLAKE_SERVER_SOCKETID_UBOX_DID,
                             SKYLAKE_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                             RESET_M2M_CTRS,
                             UNCORE_TOPOLOGY_INFO_NODE_M2M,
                             &unc_m2m_skl_callback);

    return;
}

/******************************************************************************************
 * @fn         static VOID unc_m2m_skl_Disable_PMU(PVOID)
 *
 * @brief      Disable the per unit global control to stop the PMU counters.
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 ******************************************************************************************/
static VOID
unc_m2m_skl_Disable_PMU (
    PVOID  param
)
{
    UNC_COMMON_PCI_Disable_PMU(param,
                               SKYLAKE_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_SKYLAKE_SERVER_M2M_COUNTERS,
                               DISABLE_SKYLAKE_SERVER_M2M_COUNTERS,
                               &unc_m2m_skl_callback);
    return;
}

/******************************************************************************************
 * @fn         static VOID unc_m2m_skl_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the EVSEL registers
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 ******************************************************************************************/
static VOID
unc_m2m_skl_Enable_PMU (
    PVOID   param
)
{
    UNC_COMMON_PCI_Enable_PMU(param,
                              SKYLAKE_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_SKYLAKE_SERVER_M2M_COUNTERS,
                              DISABLE_SKYLAKE_SERVER_M2M_COUNTERS,
                              &unc_m2m_skl_callback);

    return;
}


/*!
 * @fn          static VOID skx_m3qpi_Scan_For_Uncore(VOID*)
 *
 * @brief       Scan for uncore box information,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
skx_m2m_Scan_For_Uncore(
    PVOID  param
)
{

    UNC_COMMON_PCI_Scan_For_Uncore(param,
                                   UNCORE_TOPOLOGY_INFO_NODE_M2M,
                                   &unc_m2m_skl_callback);
    return;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  skx_m2m_dispatch =
{
    NULL,                                // initialize
    NULL,                                // destroy
    unc_m2m_skl_Write_PMU,              // write
    unc_m2m_skl_Disable_PMU,            // freeze
    unc_m2m_skl_Enable_PMU,             // restart
    UNC_COMMON_PCI_Read_PMU_Data,        // read
    NULL,                                // check for overflow
    NULL,                                //swap group
    NULL,                                //read lbrs
    NULL,                                //cleanup
    NULL,                                //hw errata
    NULL,                                //read power
    NULL,                                //check overflow errata
    UNC_COMMON_Read_Counts,              //read counts
    NULL,                                //check overflow gp errata
    NULL,                                // read_ro
    NULL,                                //platform info
    UNC_COMMON_PCI_Trigger_Read,         // trigger read
    skx_m2m_Scan_For_Uncore              // scan for uncore
};
