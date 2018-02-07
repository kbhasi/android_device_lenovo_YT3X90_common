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
#include "skx_unc_irp_msr.h"
#include "ecb_iterators.h"
#include "unc_common.h"


/***************************************************************************/
static DRV_BOOL
unc_irp_skx_is_Unit_Ctl (
    U32 msr_addr
)
{
    return (IS_THIS_SKYLAKE_SERVER_IRP_BOX_CTL_MSR(msr_addr));
}

static DRV_BOOL
unc_irp_skx_is_PMON_Ctl (
    U32 msr_addr
)
{
    return (IS_THIS_SKYLAKE_SERVER_IRP_BOX_EVSEL_CTL_MSR(msr_addr));
}


DEVICE_CALLBACK_NODE  unc_irp_skx_callback = {
    NULL,
    NULL,
    unc_irp_skx_is_Unit_Ctl,
    unc_irp_skx_is_PMON_Ctl
};


/*!
 * @fn          static VOID unc_irp_skx_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
unc_irp_skx_Write_PMU (
    VOID  *param
)
{
    U32              dev_idx       = *((U32*)param);
    U64              value         = 0;
    U32              this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE        pcpu          = &pcb[this_cpu];
    DEVICE_CALLBACK  callback      = &unc_irp_skx_callback;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    if (SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR) {
        SYS_Write_MSR(SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR, 0LL);
    }
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        /*
        * Writing the GLOBAL Control register enables the PMU to start counting.
        * So write 0 into the register to prevent any counting from starting.
        */
        if (ECB_entries_reg_id(pecb,i) == SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR) {
            continue;
        }
        if (RESET_IRP_MSR_CTRS                     &&
            callback                               &&
            callback->is_Unit_Ctl                  &&
            callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {

            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), RESET_IRP_MSR_CTRS);
            SEP_PRINT_DEBUG("common_sbox_Write_PMU Read reg = 0x%x --- value 0x%x\n",
                                     ECB_entries_reg_id(pecb,i), value);
            value = 0x0;
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
            SEP_PRINT_DEBUG("common_sbox_Write_PMU reg = 0x%x --- value 0x%x\n",
                                     ECB_entries_reg_id(pecb,i), value);
            continue;
        }

        SYS_Write_MSR(ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));
        SEP_PRINT_DEBUG("UNC_COMMON_MSR_Write_PMU Event_Data_reg = 0x%x --- value 0x%llx\n",
                        ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));

        // this is needed for overflow detection of the accumulators.
        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
            LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,i);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}
/*!
 * @fn         static VOID unc_irp_skx_Disable_PMU(PVOID)
 *
 * @brief      Set box level control register bit to stop PMU counters
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
unc_irp_skx_Disable_PMU (
    PVOID  param
)
{
    UNC_COMMON_MSR_Disable_PMU(param,
                               SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR,
                               DISABLE_SKYLAKE_SERVER_IRP_COUNTERS,
                               0,
                               &unc_irp_skx_callback);
    return;
}

/*!
 * @fn         static VOID unc_irp_skx_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
unc_irp_skx_Enable_PMU (
    PVOID   param
)
{
    UNC_COMMON_MSR_Enable_PMU(param,
                              SKYLAKE_SERVER_UBOX_UNIT_GLOBAL_CONTROL_MSR,
                              0,
                              DISABLE_SKYLAKE_SERVER_IRP_COUNTERS,
                              ENABLE_SKYLAKE_SERVER_IRP_COUNTERS,
                              &unc_irp_skx_callback);
    return;
}


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  skylake_server_irp_dispatch =
{
    NULL,                             // initialize
    NULL,                             // destroy
    unc_irp_skx_Write_PMU,            // write
    unc_irp_skx_Disable_PMU,          // freeze
    unc_irp_skx_Enable_PMU,           // restart
    UNC_COMMON_MSR_Read_PMU_Data,     // read
    NULL,                             // check for overflow
    NULL,                             // swap group
    NULL,                             // read lbrs
    NULL,                             // cleanup
    NULL,                             // hw errata
    NULL,                             // read power
    NULL,                             // check overflow errata
    UNC_COMMON_Read_Counts,           // read counts
    NULL,                             // check overflow gp errata
    NULL,                             // read_ro
    NULL,                             // platform info
    UNC_COMMON_MSR_Trigger_Read,      // trigger read
    NULL                              // scan for uncore
};


