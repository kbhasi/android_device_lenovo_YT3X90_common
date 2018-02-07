/*COPYRIGHT**
    Copyright (C) 2005-2016 Intel Corporation.  All Rights Reserved.

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
#include "unc_common.h"
#include "jkt_unc_ubox.h"
#include "jkt_unc_imc.h"
#include "ivt_unc_ubox.h"
#include "ivt_unc_imc.h"
#include "wsx_unc_imc.h"
#include "hsx_unc_imc.h"
#include "hsx_unc_ubox.h"
#include "knl_unc_imc.h"
#include "skx_unc_imc.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern U64           *read_counter_info;
extern DRV_CONFIG     pcfg;
 

#define INVALID_VALUE 0xFFFFFFFF

/*
 * device specific functions
 */


/***********************************************************************/
/*
 * dispatch function for JKT IMC
 *
 **********************************************************************/
static DRV_BOOL
unc_imc_jkt_is_Valid(
    U32  device_id
)
{
    if ((device_id != JKTUNC_IMC0_DID) &&
        (device_id != JKTUNC_IMC1_DID) &&
        (device_id != JKTUNC_IMC2_DID) &&
        (device_id != JKTUNC_IMC3_DID)) {
         return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_jkt_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_jkt_is_Valid(device_id);
}

DEVICE_CALLBACK_NODE  unc_imc_jkt_callback = {
    unc_imc_jkt_is_Valid,
    unc_imc_jkt_is_Valid_For_Write,
    NULL,
    NULL
};

static VOID
unc_imc_jkt_Write_PMU(
    PVOID param
)
{    
     UNC_COMMON_PCI_Write_PMU(param, JKTUNC_SOCKETID_UBOX_DID, 0, 0, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_jkt_callback);
}

static VOID
unc_imc_jkt_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_jkt_callback);
}


/***********************************************************************
 *
 * dispatch function for IVT IMC
 *
 **********************************************************************/
static DRV_BOOL
unc_imc_ivt_is_Valid(
    U32  device_id
)
{
    if ((device_id != IVTUNC_IMC0_DID) &&
        (device_id != IVTUNC_IMC1_DID) &&
        (device_id != IVTUNC_IMC2_DID) &&
        (device_id != IVTUNC_IMC3_DID) &&
        (device_id != IVTUNC_IMC4_DID) &&
        (device_id != IVTUNC_IMC5_DID) &&
        (device_id != IVTUNC_IMC6_DID) &&
        (device_id != IVTUNC_IMC7_DID)) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL 
unc_imc_ivt_is_Valid_For_Write (
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_ivt_is_Valid(device_id);
}


static DRV_BOOL
unc_imc_ivt_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_BOX_MC_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_imc_ivt_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_MC_PCI_PMON_CTL(reg_id));
}

DEVICE_CALLBACK_NODE  unc_imc_ivt_callback = {
    unc_imc_ivt_is_Valid,
    unc_imc_ivt_is_Valid_For_Write,
    unc_imc_ivt_is_Unit_Ctl,
    unc_imc_ivt_is_PMON_Ctl
};


static VOID
unc_imc_ivt_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param, 
                         IVTUNC_SOCKETID_UBOX_DID, 
                         0, 
                         RESET_IMC_CTRS,
                         UNCORE_TOPOLOGY_INFO_NODE_IMC,
                         &unc_imc_ivt_callback); 
    return;
}

static VOID
unc_imc_ivt_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param, 
                              IVYTOWN_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_IMC_COUNTERS,
                              DISABLE_IMC_COUNTERS,
                              &unc_imc_ivt_callback); 
    return;
}

static VOID
unc_imc_ivt_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param, 
                               IVYTOWN_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_IMC_COUNTERS,
                               DISABLE_IMC_COUNTERS,
                               &unc_imc_ivt_callback); 
    return;
}

static VOID 
unc_imc_ivt_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_ivt_callback);
}



/***********************************************************************
 *
 * dispatch function for Hsx IMC
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_hsx_is_Valid(
    U32  device_id
)
{
    if ((device_id != HASWELL_SERVER_IMC0_DID) &&
        (device_id != HASWELL_SERVER_IMC1_DID) &&
        (device_id != HASWELL_SERVER_IMC2_DID) &&
        (device_id != HASWELL_SERVER_IMC3_DID) &&
        (device_id != HASWELL_SERVER_IMC4_DID) &&
        (device_id != HASWELL_SERVER_IMC5_DID) &&
        (device_id != HASWELL_SERVER_IMC6_DID) &&
        (device_id != HASWELL_SERVER_IMC7_DID)) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_hsx_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_hsx_is_Valid(device_id);
}

static DRV_BOOL
unc_imc_hsx_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_HASWELL_SERVER_BOX_MC_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_imc_hsx_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_HASWELL_SERVER_MC_PCI_PMON_CTL(reg_id));
}

DEVICE_CALLBACK_NODE  unc_imc_hsx_callback = {
    unc_imc_hsx_is_Valid,
    unc_imc_hsx_is_Valid_For_Write,
    unc_imc_hsx_is_Unit_Ctl,
    unc_imc_hsx_is_PMON_Ctl
};


static VOID
unc_imc_hsx_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param, 
                          HASWELL_SERVER_SOCKETID_UBOX_DID, 
                          HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR, 
                          RESET_IMC_CTRS,
                          UNCORE_TOPOLOGY_INFO_NODE_IMC,
                          &unc_imc_hsx_callback);
    return;
}

static VOID
unc_imc_hsx_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param, 
                              HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                              DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                              &unc_imc_hsx_callback);
    return;
}

static VOID
unc_imc_hsx_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param, 
                               HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                               DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                               &unc_imc_hsx_callback);
    return;
}

static VOID
unc_imc_hsx_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_hsx_callback);
}


/***********************************************************************
 *
 * dispatch function for BDX-DE IMC
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_bdw_de_is_Valid(
    U32  device_id
)
{
    if ((device_id != BROADWELL_DE_IMC0_DID) &&
        (device_id != BROADWELL_DE_IMC1_DID)) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_bdw_de_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_bdw_de_is_Valid(device_id);
}

DEVICE_CALLBACK_NODE  unc_imc_bdw_de_callback = {
    unc_imc_bdw_de_is_Valid,
    unc_imc_bdw_de_is_Valid_For_Write,
    unc_imc_hsx_is_Unit_Ctl,
    unc_imc_hsx_is_PMON_Ctl
};


static VOID
unc_imc_bdw_de_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param,
                          BROADWELL_DE_SOCKETID_UBOX_DID,
                          HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                          RESET_IMC_CTRS,
                          UNCORE_TOPOLOGY_INFO_NODE_IMC,
                          &unc_imc_bdw_de_callback);
    return;
}

static VOID
unc_imc_bdw_de_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param,
                              HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                              DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                              &unc_imc_bdw_de_callback);
    return;
}

static VOID
unc_imc_bdw_de_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param,
                               HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                               DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                               &unc_imc_bdw_de_callback);
    return;
}


static VOID
unc_imc_bdw_de_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_bdw_de_callback);
}


/***********************************************************************
 *
 * dispatch function for KNL IMC UCLK
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_uclk_knl_is_Valid(
    U32  device_id
)
{
    if (device_id != KNL_IMC_UCLK_DID) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_uclk_knl_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_uclk_knl_is_Valid(device_id);
}

static DRV_BOOL
unc_imc_uclk_knl_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_BOX_MC_UCLK_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_imc_uclk_knl_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_MC_UCLK_PCI_PMON_CTL(reg_id));
}

DEVICE_CALLBACK_NODE  unc_imc_uclk_knl_callback = {
    unc_imc_uclk_knl_is_Valid,
    unc_imc_uclk_knl_is_Valid_For_Write,
    unc_imc_uclk_knl_is_Unit_Ctl,
    unc_imc_uclk_knl_is_PMON_Ctl
};


static VOID
unc_imc_uclk_knl_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param, 
                             KNL_SOCKETID_UBOX_DID, 
                             KNL_UBOX_GLOBAL_CONTROL_MSR, 
                             RESET_IMC_CTRS,
                             UNCORE_TOPOLOGY_INFO_NODE_IMC,
                             &unc_imc_uclk_knl_callback);
    return;
}

static VOID
unc_imc_uclk_knl_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param, 
                              KNL_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_KNL_IMC_COUNTERS,
                              DISABLE_KNL_IMC_COUNTERS,
                              &unc_imc_uclk_knl_callback);
    return;
}

static VOID
unc_imc_uclk_knl_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param, 
                               KNL_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_KNL_IMC_COUNTERS,
                               DISABLE_KNL_IMC_COUNTERS,
                               &unc_imc_uclk_knl_callback);
    return;
}

static VOID
unc_imc_uclk_knl_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC_UCLK, &unc_imc_uclk_knl_callback);
}


/***********************************************************************
 *
 * dispatch function for KNL IMC CHX
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_chx_knl_is_Valid(
    U32  device_id
)
{
    if (device_id != KNL_IMC_CHX_DID) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_chx_knl_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_chx_knl_is_Valid(device_id);
}

static DRV_BOOL
unc_imc_chx_knl_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_BOX_MC_CHX_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_imc_chx_knl_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_KNL_MC_CHX_PCI_PMON_CTL(reg_id));
}

DEVICE_CALLBACK_NODE  unc_imc_chx_knl_callback = {
    unc_imc_chx_knl_is_Valid,
    unc_imc_chx_knl_is_Valid_For_Write,
    unc_imc_chx_knl_is_Unit_Ctl,
    unc_imc_chx_knl_is_PMON_Ctl
};


static VOID
unc_imc_chx_knl_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param, 
                             KNL_SOCKETID_UBOX_DID, 
                             KNL_UBOX_GLOBAL_CONTROL_MSR, 
                             RESET_IMC_CTRS,
                             UNCORE_TOPOLOGY_INFO_NODE_IMC_UCLK,
                             &unc_imc_chx_knl_callback);
    return;
}

static VOID
unc_imc_chx_knl_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param, 
                              KNL_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_KNL_IMC_COUNTERS,
                              DISABLE_KNL_IMC_COUNTERS,
                              &unc_imc_chx_knl_callback);
    return;
}

static VOID
unc_imc_chx_knl_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param, 
                               KNL_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_KNL_IMC_COUNTERS,
                               DISABLE_KNL_IMC_COUNTERS,
                               &unc_imc_chx_knl_callback);
    return;
}

static VOID
unc_imc_chx_knl_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_chx_knl_callback);
}


/*************************************************/

/*!
 * @fn          static VOID unc_imc_Get_DIMM_Info(VOID)
 *
 * @brief       This code reads appropriate pci_config regs and returns dimm info
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
 
static VOID
unc_imc_Get_DIMM_Info (
    U64                 ubox_did,
    U64                 device_num1,
    U64                 device_num2,
    DRV_DIMM_INFO_NODE *dimm_info
)
{
    U32 pci_address;
    U32 bus_num    = 0;
    U32 func_num   = 2;
    U32 channel_num = 0;
    U32 i;
    U32 dimm_idx   = 0;
    U32 num_pkgs   = num_packages;
    U32 bus_map_index = 0;
 
    if (unc_package_to_bus_map == NULL) {
            return;
        }

    if (num_packages > MAX_PACKAGES) {
        SEP_PRINT_ERROR("unc_imc_Get_DIMM_Info: num_packages %d exceeds MAX_PACKAGE %d, only retrieve info for %d packages\n",
                         num_packages, MAX_PACKAGES, MAX_PACKAGES);
        num_pkgs = MAX_PACKAGES;
    }
 
    for (i = 0; i < num_pkgs; i++) {
        bus_map_index = i*MAX_DEVICES+UNCORE_TOPOLOGY_INFO_NODE_IMC;
        if (unc_package_to_bus_map[bus_map_index]) {
            bus_num = unc_package_to_bus_map[bus_map_index]->number;
        }
        else {
            continue;
        }
 
        // channels 0 - 3
        channel_num = 0;
        for (func_num = 2, channel_num = 0; channel_num < 4; func_num++, channel_num++) {
            // now get the dimm info:
            DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
            DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
            DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 0;
            pci_address = FORM_PCI_ADDR(bus_num, device_num1, func_num, 0x80);
            DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
 
            DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
            DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
            DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 1;
            pci_address = FORM_PCI_ADDR(bus_num, device_num1, func_num, 0x84);
            DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
 
            DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
            DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
            DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 2;
            pci_address = FORM_PCI_ADDR(bus_num, device_num1, func_num, 0x88);
            DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
        }
 
        // channels 4 - 7
        if (device_num2) {
            // now get the dimm info:
            for (func_num = 2; channel_num < 8; func_num++, channel_num++) {
                DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
                DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
                DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 0;
                pci_address = FORM_PCI_ADDR(bus_num, device_num2, func_num, 0x80);
                DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
 
                DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
                DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
                DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 1;
                pci_address = FORM_PCI_ADDR(bus_num, device_num2, func_num, 0x84);
                DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
 
                DRV_DIMM_INFO_platform_id(&dimm_info[dimm_idx]) = i;
                DRV_DIMM_INFO_channel_num(&dimm_info[dimm_idx]) = channel_num;
                DRV_DIMM_INFO_rank_num(&dimm_info[dimm_idx])    = 2;
                pci_address = FORM_PCI_ADDR(bus_num, device_num2, func_num, 0x88);
                DRV_DIMM_INFO_value(&dimm_info[dimm_idx++])     = PCI_Read_Ulong_Valid(pci_address, INVALID_VALUE);
            }
        }
    }
 
    return;
}
 
/* ------------------------------------------------------------------------- */
/*!
 * @fn          PVOID unc_imc_Platform_Info
 *
 * @brief       Reads and returns DIMM info
 *
 * @param       Platform_Info data structure
 *
 * @return      VOID
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static VOID
unc_imc_Platform_Info (
    PVOID               param,
    U32                 ubox_did,
    U32                 device_num1,
    U32                 device_num2
)
{
    DRV_PLATFORM_INFO      platform_data  = (DRV_PLATFORM_INFO)param;
    DRV_DIMM_INFO          dimm_info      = DRV_PLATFORM_INFO_dimm_info(platform_data);
 
    if (!platform_data) {
        return;
    }
 
    unc_imc_Get_DIMM_Info(ubox_did, device_num1, device_num2, dimm_info);
    return;
}

 
 
static VOID 
unc_imc_jkt_Platform_Info(
    PVOID param
)
{
    unc_imc_Platform_Info(param, JKTUNC_SOCKETID_UBOX_DID, 15, 0);
}
static VOID 
unc_imc_ivt_Platform_Info(
    PVOID param
)
{
      unc_imc_Platform_Info(param, IVYTOWN_SOCKETID_UBOX_DID, 15, 29);
}
static VOID 
unc_imc_hsx_Platform_Info(
    PVOID param
)
{
    unc_imc_Platform_Info(param,  HASWELL_SERVER_SOCKETID_UBOX_DID, 19, 22);
}

static VOID 
unc_imc_knl_Platform_Info(
    PVOID param
)
{
    unc_imc_Platform_Info(param,  KNL_SOCKETID_UBOX_DID, 19, 22);
}

/***********************************************************************
 *
 * dispatch function for Wsmex IMC
 *
 ***********************************************************************/


static VOID
unc_imc_wsmex_Write_PMU (
    VOID  *param
)
{
    UNC_COMMON_MSR_Write_PMU(param, IMC_PERF_GLOBAL_CTRL, (unsigned long long)IMC_GLOBAL_DISABLE, 0, NULL);
    return;
}

static VOID
unc_imc_wsmex_Disable_PMU (
    PVOID  param
)
{
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
 
    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
    SYS_Write_MSR(IMC_PERF_GLOBAL_CTRL, (unsigned long long)IMC_GLOBAL_DISABLE);
    return;
}

static VOID
unc_imc_wsmex_Enable_PMU (
    PVOID   param
)
{
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
 
    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
 
    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_RUNNING) {
        SYS_Write_MSR(IMC_PERF_GLOBAL_CTRL, (unsigned long long)IMC_GLOBAL_ENABLE);
    }
    return;
}

 
/*!
 * @fn         static VOID unc_imc_wsmex_Read_PMU_Data(PVOID)
 *
 * @brief      Read all the data MSR's into a buffer.
 *             Called by the interrupt handler
 *
 * @param      buffer      - pointer to the output buffer
 *             start       - position of the first entry
 *             stop        - last possible entry to this buffer
 *
 * @return     None
 */
static VOID
unc_imc_wsmex_Read_PMU_Data(
    PVOID   param
)
{
    S32       start_index, j;
    U64      *buffer    = read_counter_info;
    U32       this_cpu  = CONTROL_THIS_CPU();
 
    start_index = DRV_CONFIG_num_events(pcfg) * this_cpu;
    SEP_PRINT_DEBUG("PMU control_data 0x%p, buffer 0x%p, j = %d\n", PMU_register_data, buffer, j);
    FOR_EACH_DATA_REG(pecb_unc,i) {
        j = start_index + ECB_entries_event_id_index(pecb_unc,i);
        buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb_unc,i));
        SEP_PRINT_DEBUG("this_cpu %d, event_id %d, value 0x%llx\n", this_cpu, i, buffer[j]);
    } END_FOR_EACH_DATA_REG;

    return;
}

static VOID
unc_imc_bdw_de_Platform_Info(
    PVOID param
)
{
    unc_imc_Platform_Info(param,  BROADWELL_DE_SOCKETID_UBOX_DID, 19, 22);
}

/***********************************************************************
 *
 * dispatch function for BDX-Server IMC
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_bdw_server_is_Valid(
    U32  device_id
)
{
    if ((device_id != BROADWELL_DE_IMC0_DID)     &&
        (device_id != BROADWELL_DE_IMC1_DID)     &&
		(device_id != BROADWELL_SERVER_IMC2_DID) &&
		(device_id != BROADWELL_SERVER_IMC3_DID) &&
		(device_id != BROADWELL_SERVER_IMC4_DID) &&
		(device_id != BROADWELL_SERVER_IMC5_DID) &&
		(device_id != BROADWELL_SERVER_IMC6_DID) &&
        (device_id != BROADWELL_SERVER_IMC7_DID) ) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_bdw_server_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_bdw_server_is_Valid(device_id);
}

DEVICE_CALLBACK_NODE  unc_imc_bdw_server_callback = {
    unc_imc_bdw_server_is_Valid,
    unc_imc_bdw_server_is_Valid_For_Write,
    unc_imc_hsx_is_Unit_Ctl,
    unc_imc_hsx_is_PMON_Ctl
};


static VOID
unc_imc_bdw_server_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param,
                          BROADWELL_DE_SOCKETID_UBOX_DID,
                          HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                          RESET_IMC_CTRS,
                          UNCORE_TOPOLOGY_INFO_NODE_IMC,
                          &unc_imc_bdw_server_callback);
    return;
}

static VOID
unc_imc_bdw_server_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param,
                              HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                              DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                              &unc_imc_bdw_server_callback);
    return;
}

static VOID
unc_imc_bdw_server_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param,
                               HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_HASWELL_SERVER_IMC_COUNTERS,
                               DISABLE_HASWELL_SERVER_IMC_COUNTERS,
                               &unc_imc_bdw_server_callback);
    return;
}



static VOID
unc_imc_bdw_server_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_bdw_server_callback);
};

/*************************************************/


/***********************************************************************
 *
 * dispatch functions for Skylake Server (SKX) IMC
 *
 ***********************************************************************/

static DRV_BOOL
unc_imc_skx_is_Valid(
    U32  device_id
)
{
    if ((device_id != SKYLAKE_SERVER_IMC0_DID) &&
        (device_id != SKYLAKE_SERVER_IMC1_DID) &&
        (device_id != SKYLAKE_SERVER_IMC2_DID) &&
        (device_id != SKYLAKE_SERVER_IMC3_DID) &&
        (device_id != SKYLAKE_SERVER_IMC4_DID) &&
        (device_id != SKYLAKE_SERVER_IMC5_DID)
        ) {
        return FALSE;
    }
    return TRUE;
}

static DRV_BOOL
unc_imc_skx_is_Valid_For_Write(
    U32  device_id,
    U32  reg_id
)
{
    return unc_imc_skx_is_Valid(device_id);
}

static DRV_BOOL
unc_imc_skx_is_Unit_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_SKYLAKE_SERVER_BOX_MC_PCI_UNIT_CTL(reg_id));
}

static DRV_BOOL
unc_imc_skx_is_PMON_Ctl(
    U32  reg_id
)
{
    return (IS_THIS_SKYLAKE_SERVER_MC_PCI_PMON_CTL(reg_id));
}

DEVICE_CALLBACK_NODE  unc_imc_skx_callback = {
    unc_imc_skx_is_Valid,
    unc_imc_skx_is_Valid_For_Write,
    unc_imc_skx_is_Unit_Ctl,
    unc_imc_skx_is_PMON_Ctl
};


/******************************************************************************************
 * @fn          static VOID unc_imc_skx_Write_PMU(VOID*)
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
unc_imc_skx_Write_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Write_PMU(param,
                          SKYLAKE_SERVER_SOCKETID_UBOX_DID,
                          HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                          RESET_IMC_CTRS,
                          UNCORE_TOPOLOGY_INFO_NODE_IMC,
                          &unc_imc_skx_callback);
    return;
}

/******************************************************************************************
 * @fn         static VOID unc_imc_skx_Enable_PMU(PVOID)
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
unc_imc_skx_Enable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Enable_PMU(param,
                              HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                              ENABLE_SKYLAKE_SERVER_IMC_COUNTERS,
                              DISABLE_SKYLAKE_SERVER_IMC_COUNTERS,
                              &unc_imc_skx_callback);
    return;
}

/******************************************************************************************
 * @fn         static VOID unc_imc_skx_Disable_PMU(PVOID)
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
unc_imc_skx_Disable_PMU(
    PVOID param
)
{
    UNC_COMMON_PCI_Disable_PMU(param,
                               HASWELL_SERVER_UBOX_GLOBAL_CONTROL_MSR,
                               ENABLE_SKYLAKE_SERVER_IMC_COUNTERS,
                               DISABLE_SKYLAKE_SERVER_IMC_COUNTERS,
                               &unc_imc_skx_callback);
    return;
}

/******************************************************************************************
 * @fn         static VOID unc_imc_skx_Scan_For_Uncore(PVOID)
 *
 * @brief      Scan uncoe box information.
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 ******************************************************************************************/
static VOID
unc_imc_skx_Scan_For_Uncore(
    PVOID param
)
{
    UNC_COMMON_PCI_Scan_For_Uncore(param, UNCORE_TOPOLOGY_INFO_NODE_IMC, &unc_imc_skx_callback);
};

/******************************************************************************************
 * @fn         static VOID unc_imc_skx_Platform_Info(PVOID)
 *
 * @brief      Get Platform information.
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 ******************************************************************************************/
static VOID
unc_imc_skx_Platform_Info(
    PVOID param
)
{
    unc_imc_skx_Scan_For_Uncore(0);
    unc_imc_Platform_Info(param,  SKYLAKE_SERVER_SOCKETID_UBOX_DID, 19, 22);
}



/*************************************************/

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  wsmexunc_imc_dispatch =
{
    NULL,                               // initialize
    NULL,                               // destroy
    unc_imc_wsmex_Write_PMU,            // write
    unc_imc_wsmex_Disable_PMU,          // freeze
    unc_imc_wsmex_Enable_PMU,           // restart
    unc_imc_wsmex_Read_PMU_Data,        // read
    NULL,                               // check for overflow
    NULL,
    NULL,
    UNC_COMMON_MSR_Clean_Up,
    NULL,
    NULL,
    NULL,
    UNC_COMMON_Read_Counts,
    NULL,                               // check_overflow_gp_errata
    NULL,                               // read_ro
    NULL,                               // platform_info
    UNC_COMMON_MSR_Trigger_Read,
    NULL                                // scan for uncore
};
 
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  jktunc_imc_dispatch =
{
    NULL,                                  // initialize
    NULL,                                  // destroy
    unc_imc_jkt_Write_PMU,                 // write
    NULL,                                  // freeze
    UNC_COMMON_Dummy_Func,                 // restart
    UNC_COMMON_PCI_Read_PMU_Data,          // read
    NULL,                                  // check for overflow
    NULL,
    NULL,
    UNC_COMMON_PCI_Clean_Up,
    NULL,
    NULL,
    NULL,
    UNC_COMMON_Read_Counts,               // read_counts
    NULL,
    NULL,
    unc_imc_jkt_Platform_Info,              // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_jkt_Scan_For_Uncore             // scan for uncore
};
 
 
 
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  ivtunc_imc_dispatch =
{
    NULL,                        // initialize
    NULL,                        // destroy
    unc_imc_ivt_Write_PMU,        // write
    unc_imc_ivt_Disable_PMU,      // freeze
    unc_imc_ivt_Enable_PMU,       // restart
    UNC_COMMON_PCI_Read_PMU_Data,   // read
    NULL,                        // check for overflow
    NULL,                        // swap_group
    NULL,                        // read_lbrs
    UNC_COMMON_PCI_Clean_Up,        // cleanup
    NULL,                        // hw_errata
    NULL,                        // read_power
    NULL,                        // check overflow errata
    UNC_COMMON_Read_Counts,      // read_counts
    NULL,                        // check overflow gp errata
    NULL,                        // read_ro
    unc_imc_ivt_Platform_Info,   // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_ivt_Scan_For_Uncore      // scan for uncore
};
 
 
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE haswell_server_imc_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_hsx_Write_PMU,             // write
    unc_imc_hsx_Disable_PMU,           // freeze
    unc_imc_hsx_Enable_PMU,            // restart
    UNC_COMMON_PCI_Read_PMU_Data,         // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,              // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,               // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_hsx_Platform_Info,         // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_hsx_Scan_For_Uncore        // scan for uncore
};
 

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE broadwell_de_imc_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_bdw_de_Write_PMU,          // write
    unc_imc_bdw_de_Disable_PMU,           // freeze
    unc_imc_bdw_de_Enable_PMU,            // restart
    UNC_COMMON_PCI_Read_PMU_Data,      // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,            // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_bdw_de_Platform_Info,      // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_bdw_de_Scan_For_Uncore     // scan for uncore
};

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE knl_imc_uclk_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_uclk_knl_Write_PMU,        // write
    unc_imc_uclk_knl_Disable_PMU,      // freeze
    unc_imc_uclk_knl_Enable_PMU,       // restart
    UNC_COMMON_PCI_Read_PMU_Data,      // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,            // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_knl_Platform_Info,         // platform info
    UNC_COMMON_PCI_Trigger_Read,                     
    unc_imc_uclk_knl_Scan_For_Uncore   // scan for uncore
};

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE knl_imc_chx_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_chx_knl_Write_PMU,         // write
    unc_imc_chx_knl_Disable_PMU,       // freeze
    unc_imc_chx_knl_Enable_PMU,        // restart
    UNC_COMMON_PCI_Read_PMU_Data,      // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,            // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_knl_Platform_Info,         // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_chx_knl_Scan_For_Uncore    // scan for uncore
};

DISPATCH_NODE broadwell_server_imc_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_bdw_server_Write_PMU,      // write
    unc_imc_bdw_server_Disable_PMU,    // freeze
    unc_imc_bdw_server_Enable_PMU,     // restart
    UNC_COMMON_PCI_Read_PMU_Data,      // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,            // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_bdw_de_Platform_Info,      // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_bdw_server_Scan_For_Uncore // scan for uncore
};


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE skx_imc_dispatch =
{
    NULL,                              // initialize
    NULL,                              // destroy
    unc_imc_skx_Write_PMU,             // write
    unc_imc_skx_Disable_PMU,           // freeze
    unc_imc_skx_Enable_PMU,            // restart
    UNC_COMMON_PCI_Read_PMU_Data,      // read
    NULL,                              // check for overflow
    NULL,                              // swap_group
    NULL,                              // read_lbrs
    UNC_COMMON_PCI_Clean_Up,           // cleanup
    NULL,                              // hw_errata
    NULL,                              // read_power
    NULL,                              // check overflow errata
    UNC_COMMON_Read_Counts,            // read_counts
    NULL,                              // check overflow gp errata
    NULL,                              // read_ro
    unc_imc_skx_Platform_Info,         // platform info
    UNC_COMMON_PCI_Trigger_Read,
    unc_imc_skx_Scan_For_Uncore        // scan for uncore
};

