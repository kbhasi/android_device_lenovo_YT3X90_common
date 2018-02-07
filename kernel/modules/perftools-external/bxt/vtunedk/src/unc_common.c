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
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern UNCORE_TOPOLOGY_INFO_NODE uncore_topology;
extern U64                      *read_counter_info;

struct pci_bus                 **unc_package_to_bus_map;


/************************************************************/
/* 
 * unc common Dispatch functions
 *
 ************************************************************/
extern  void
UNC_COMMON_Dummy_Func (
    PVOID  param
)
{
    return;
}

/*!
 * @fn       extern void UNC_COMMON_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *           Uncore PMU does not support sampling, i.e. ignore the id parameter.
 */
extern  VOID
UNC_COMMON_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data;
    ECB             pecb;
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = core_to_package_map[this_cpu];
    U64            *read_buf;
    U32             index;
    U32             i;

    //Read in the counts from each group into temporary buffer
    for (i = 0; i < LWPMU_DEVICE_em_groups_count(&devices[id]); i++) {
        index = 0;
        if (i == LWPMU_DEVICE_to_read_data(&devices[id])[package_num][0]) { //data is current
            read_buf = &LWPMU_DEVICE_to_read_data(&devices[id])[package_num][1]; 
        }
        else {
            read_buf = LWPMU_DEVICE_acc_value(&devices[id])[package_num][i];
        }
        pecb = LWPMU_DEVICE_PMU_register_data(&devices[id])[i];
        // Write GroupID
        data    = (U64*)((S8*)param + ECB_group_offset(pecb));
        *data   = i + 1;
        FOR_EACH_DATA_REG_UNC_VER2(pecb, id, idx) {
            data  = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,idx));
            *data = read_buf[index];
            index++;
        } END_FOR_EACH_DATA_REG_UNC_VER2;
    }

    return;
}

/************************************************************/
/*
 * UNC common PCI  based API
 *
 ************************************************************/

/*!
 * @fn          OS_STATUS UNC_COMMON_Do_Bus_to_Socket_Map(VOID)
 * 
 * @brief       This code discovers which package's data is read off of which bus.
 *
 * @param       None
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 *     This probably will move to the UBOX once that is programmed.
 */
OS_STATUS
UNC_COMMON_Do_Bus_to_Socket_Map(
    U32 uncore_did,
    U32 dev_node,
    U32 busno
)
{
    struct pci_dev  *pdev       = NULL;
    U32              package_num = 0;
    U32              dev         = 0;

    if (unc_package_to_bus_map == NULL) {
        unc_package_to_bus_map = CONTROL_Allocate_Memory(num_packages * MAX_DEVICES * sizeof(struct pci_bus*));
        if (unc_package_to_bus_map == NULL) {
        SEP_PRINT_DEBUG("UNC_COMMON_Do_Bus_to_Socket_Map allocated NULL by CONTROL_Allocate_Memory\n");
        return OS_NO_MEM;
    }
        for (dev = 0; dev < (num_packages * MAX_DEVICES); dev++) {
            unc_package_to_bus_map[dev] = 0;
        }
    }

    pdev = pci_get_device(DRV_IS_PCI_VENDOR_ID_INTEL, uncore_did, NULL);
    while (pdev != NULL) {
        if (pdev->bus->number == busno) {
            SEP_PRINT_DEBUG("BUS MAP package=%d device_id=0x%x dev_node=%d bus=0x%x\n", package_num, uncore_did, dev_node, pdev->bus->number);
            break;
        }
        pdev = pci_get_device(PCI_VENDOR_ID_INTEL, uncore_did, pdev);
    }
    if (!pdev) {
        return OS_FAULT;
    }

    while (package_num < num_packages && unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node] != NULL) {
        if (busno == unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node]->number) {
            break;
        }
        package_num++;
    }
    if (package_num < num_packages) {
        unc_package_to_bus_map[package_num*MAX_DEVICES+dev_node] = pdev->bus;
    }
    else {
        return OS_FAULT;
    }
    
    return OS_SUCCESS;
}


/*!
 * @fn          extern VOID UNC_COMMON_PCI_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_PCI_Write_PMU (
    PVOID            param,
    U32              ubox_did,
    U32              control_msr,
    U32              ctl_val,
    U32              dev_node,
    DEVICE_CALLBACK  callback
)
{
    U32                        pci_address;
    U32                        device_id;
    U32                        dev_idx       = *((U32*)param);
    U32                        value;
    U32                        vendor_id;
    U32                        this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE                  pcpu          = &pcb[this_cpu];
    U32                        package_num   = 0;
    U32                        bus_map_index = 0;


    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    // first, figure out which package maps to which bus

    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]) = dev_node;
    FOR_EACH_REG_ENTRY_UNC(pecb,dev_idx,idx) {
        if (control_msr  && (ECB_entries_reg_id(pecb,idx) == control_msr)) {
             //Check if we need to zero this MSR out
             SYS_Write_MSR(ECB_entries_reg_id(pecb,idx), 0LL);
             SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU wrote GLOBAL_CONTROL_MSR 0x%x\n", control_msr);
             continue;
        }

        // otherwise, we have a valid entry
        // now we just need to find the corresponding bus #
        ECB_entries_bus_no(pecb,idx) = unc_package_to_bus_map[bus_map_index]->number;
        pci_address = FORM_PCI_ADDR(ECB_entries_bus_no(pecb,idx),
                                    ECB_entries_dev_no(pecb,idx),
                                    ECB_entries_func_no(pecb,idx),
                                    0);
        value = PCI_Read_Ulong(pci_address);

        CHECK_IF_GENUINE_INTEL_DEVICE(value, vendor_id, device_id);
           
        if (callback                              &&
            callback->is_Valid_For_Write          &&
            !(callback->is_Valid_For_Write(device_id, ECB_entries_reg_id(pecb,idx)))) {
            continue;
        }

        if (ctl_val                                  &&
            callback                                 &&
            callback->is_Unit_Ctl                    &&
            (ECB_entries_reg_type(pecb,idx) == CCCR) &&
             callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,idx))) {
             value = ctl_val;
             // busno can not be stored in ECB because different sockets have different bus no.
             PCI_Write(unc_package_to_bus_map[bus_map_index],
                       ECB_entries_dev_no(pecb,idx),
                       ECB_entries_func_no(pecb,idx),
                       ECB_entries_reg_id(pecb,idx),
                       value);
             SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU cpu=%d, reg = 0x%x --- value 0x%x\n",
                             this_cpu, ECB_entries_reg_id(pecb,idx), value);

             // Writing 0 to guard against sticky reset bit
             PCI_Write(unc_package_to_bus_map[bus_map_index],
                       ECB_entries_dev_no(pecb,idx),
                       ECB_entries_func_no(pecb,idx),
                       ECB_entries_reg_id(pecb,idx),
                       0);
             continue;
        }

        // now program at the corresponding offset
        PCI_Write(unc_package_to_bus_map[bus_map_index],
                  ECB_entries_dev_no(pecb,idx),
                  ECB_entries_func_no(pecb,idx),
                  ECB_entries_reg_id(pecb,idx),
                  (U32)ECB_entries_reg_value(pecb,idx));


        SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU cpu=%d, reg = 0x%x --- value 0x%x\n",
                             this_cpu, ECB_entries_reg_id(pecb,idx), (U32)ECB_entries_reg_value(pecb,idx));

        // we're zeroing out a data register, which is 48 bits long
        // we need to zero out the upper bits as well
        if (ECB_entries_reg_type(pecb,idx) == DATA) {
            PCI_Write(unc_package_to_bus_map[bus_map_index],
                      ECB_entries_dev_no(pecb,idx),
                      ECB_entries_func_no(pecb,idx),
                      (ECB_entries_reg_id(pecb,idx) + NEXT_ADDR_OFFSET),
                      (U32)ECB_entries_reg_value(pecb,idx));

            SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU cpu=%d, reg = 0x%x --- value 0x%x\n",
                             this_cpu, ECB_entries_reg_id(pecb,idx), (U32)ECB_entries_reg_value(pecb,idx));
        }

            // this is needed for overflow detection of the accumulators.
        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
             LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,idx);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn         static VOID UNC_COMMON_PCI_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the EVSEL registers
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_PCI_Enable_PMU (
    PVOID               param,
    U32                 control_msr,
    U32                 enable_val,
    U32                 disable_val,
    DEVICE_CALLBACK     callback
)
{
    U32            dev_idx       = *((U32 *)param);
    U32            value         = 0;
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U32            package_num   = 0;
    U32            dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32            bus_map_index = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    package_num        = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (ECB_entries_reg_id(pecb,i) == control_msr) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));
            SEP_PRINT_DEBUG("UNC_COMMON_PCI_Write_PMU wrote GLOBAL_CONTROL_MSR 0x%x val=0x%x\n",
                         control_msr, ECB_entries_reg_value(pecb,i));
            continue;
        }
        if (callback                               &&
            callback->is_PMON_Ctl                  &&
            (ECB_entries_reg_type(pecb,i) == CCCR) &&
             callback->is_PMON_Ctl(ECB_entries_reg_id(pecb,i))) {
             value = enable_val | ECB_entries_reg_value(pecb,i);
             PCI_Write(unc_package_to_bus_map[bus_map_index],
                       ECB_entries_dev_no(pecb,i),
                       ECB_entries_func_no(pecb,i),
                       ECB_entries_reg_id(pecb,i),
                       value);
             SEP_PRINT_DEBUG("UNC_COMMON_PCI_Enable_PMU Event_reg = 0x%x --- value 0x%x\n",
                          ECB_entries_reg_id(pecb,i), value);
             continue;
        }
        if (disable_val                            &&
            callback                               &&
            callback->is_Unit_Ctl                  &&
            callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {
            value = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,i),
                             ECB_entries_func_no(pecb,i),
                             ECB_entries_reg_id(pecb,i));
            value &= ~(disable_val);
            PCI_Write(unc_package_to_bus_map[bus_map_index],
                      ECB_entries_dev_no(pecb,i),
                      ECB_entries_func_no(pecb,i),
                      ECB_entries_reg_id(pecb,i),
                      value);
            SEP_PRINT_DEBUG("UNC_COMMON_PCI_Enable_PMU Event_reg = 0x%x --- value 0x%x\n",
                         ECB_entries_reg_id(pecb,i), value);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn           extern VOID UNC_COMMON_PCI_Disable_PMU(PVOID)
 *
 * @brief        Disable the per unit global control to stop the PMU counters.
 *
 * @param        Device Index of this PMU unit
 * @control_msr  Control MSR address
 * @enable_val   If counter freeze bit does not work, counter enable bit should be cleared
 * @disable_val  Disable collection
 *
 * @return       None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_PCI_Disable_PMU (
    PVOID               param,
    U32                 control_msr,
    U32                 enable_val,
    U32                 disable_val,
    DEVICE_CALLBACK     callback
)
{
    U32            dev_idx       = *((U32 *)param);
    U32            value;
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
    U32            package_num   = 0;
    U32            dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32            bus_map_index = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    package_num        = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (control_msr && (ECB_entries_reg_id(pecb,i) == control_msr)) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), 0LL);
            SEP_PRINT_DEBUG("UNC_COMMON_PCI_Disable_PMU wrote GLOBAL_CONTROL_MSR 0x%x\n", control_msr);
            continue;
        }
        if (callback) {
            // The enable bit must be cleared when the PMU freeze is not working
            if (enable_val && callback->is_PMON_Ctl    &&
                (ECB_entries_reg_type(pecb,i) == CCCR) &&
                callback->is_PMON_Ctl(ECB_entries_reg_id(pecb,i))) {
                value = (~enable_val) & ECB_entries_reg_value(pecb,i);
                PCI_Write(unc_package_to_bus_map[bus_map_index],
                          ECB_entries_dev_no(pecb,i),
                          ECB_entries_func_no(pecb,i),
                          ECB_entries_reg_id(pecb,i),
                          value);
                SEP_PRINT_DEBUG("UNC_COMMON_PCI_Disable_PMU cpu=%d, Event_reg = 0x%x --- value 0x%x\n",
                         this_cpu, ECB_entries_reg_id(pecb,i), value);
            }
            else if (callback->is_Unit_Ctl                   &&
                     (ECB_entries_reg_type(pecb,i) == CCCR)  &&
                     callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {
                value = disable_val | (U32)ECB_entries_reg_value(pecb,i);
                PCI_Write(unc_package_to_bus_map[bus_map_index],
                          ECB_entries_dev_no(pecb,i),
                          ECB_entries_func_no(pecb,i),
                          ECB_entries_reg_id(pecb,i),
                          value);
                SEP_PRINT_DEBUG("UNC_COMMON_PCI_Disable_PMU cpu=%d, Event_Data_reg = 0x%x --- value 0x%x\n",
                         this_cpu, ECB_entries_reg_id(pecb,i), value);
            }
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn         extern VOID UNC_COMMON_PCI_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
extern void
UNC_COMMON_PCI_Clean_Up (
    VOID   *param
)
{
    if (unc_package_to_bus_map) {
        unc_package_to_bus_map = CONTROL_Free_Memory(unc_package_to_bus_map);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       extern void UNC_COMMON_PCI_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *           Uncore PMU does not support sampling, i.e. ignore the id parameter.
 */
extern  VOID
UNC_COMMON_PCI_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data       = (U64*) param;
    U32             cur_grp    = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB             pecb       = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U64             value_low  = 0;
    U64             value_high = 0;
    U64             value      = 0;
    U32             index      = 0;
    U64             diff       = 0;
    U32             this_cpu      = CONTROL_THIS_CPU();
    U32             package_num   = core_to_package_map[this_cpu];
    U32             dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[id]);
    U32             bus_map_index = package_num*MAX_DEVICES + dev_node;

    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }

    // Write GroupID
    data    = (U64*)((S8*)data + ECB_group_offset(pecb));
    *data   = cur_grp + 1;

    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC(pecb, id, i) {
        data  = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,i));
        // read lower 4 bytes
        value_low = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,i),
                             ECB_entries_func_no(pecb,i),
                             ECB_entries_reg_id(pecb,i));
        value = LOWER_4_BYTES_MASK & value_low;

        // read upper 4 bytes
        value_high = PCI_Read(unc_package_to_bus_map[bus_map_index],
                              ECB_entries_dev_no(pecb,i),
                              ECB_entries_func_no(pecb,i),
                              (ECB_entries_reg_id(pecb,i) + NEXT_ADDR_OFFSET));
        value |= value_high << NEXT_ADDR_SHIFT;
        //check for overflow
        if (value < LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index]) {
            diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index];
            diff += value;
        }
        else {
            diff = value - LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index];
        }
        LWPMU_DEVICE_acc_per_thread(&devices[id])[this_cpu][index] += diff;
        LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index] = value;
        *data = LWPMU_DEVICE_acc_per_thread(&devices[id])[this_cpu][index];
        index++;
    } END_FOR_EACH_DATA_REG_UNC;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       void UNC_COMMON_PCI_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
extern  VOID
UNC_COMMON_PCI_Trigger_Read (
    U32    id
)
{
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = core_to_package_map[this_cpu];
    U32             dev_node;
    U32             bus_map_index;
    U32             cur_grp;
    ECB             pecb;
    CPU_STATE       pcpu                = &pcb[this_cpu];
    U32             index               = 0;
    U64             value_low           = 0;
    U64             value_high          = 0;
    U64             diff                = 0;
    U64             value;
    U64            *temp;

    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_UNINITIALIZED ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_IDLE          ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_RESERVED      ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_PREPARE_STOP  ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_STOPPED) {
        return;
    }

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    dev_node      = LWPMU_DEVICE_pci_dev_node_index(&devices[id]);
    bus_map_index = package_num*MAX_DEVICES + dev_node;
    cur_grp       = LWPMU_DEVICE_cur_group(&devices[id]);
    pecb          = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC_VER2(pecb, id, i) {
        // read lower 4 bytes
        value_low = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,i),
                             ECB_entries_func_no(pecb,i),
                             ECB_entries_reg_id(pecb,i));
        value = LOWER_4_BYTES_MASK & value_low;

        // read upper 4 bytes
        value_high = PCI_Read(unc_package_to_bus_map[bus_map_index],
                              ECB_entries_dev_no(pecb,i),
                              ECB_entries_func_no(pecb,i),
                              (ECB_entries_reg_id(pecb,i) + NEXT_ADDR_OFFSET));
        value |= value_high << NEXT_ADDR_SHIFT;
        //check for overflow
        if (value < LWPMU_DEVICE_prev_value(&devices[id])[package_num][index]) {
            diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
            diff += value;
        }
        else {
            diff = value - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
        }
        LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index] += diff;
        LWPMU_DEVICE_prev_value(&devices[id])[package_num][index] = value;
        LWPMU_DEVICE_current_data(&devices[id])[package_num][index+1] = LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index];
        index++;
    } END_FOR_EACH_DATA_REG_UNC_VER2;
    LWPMU_DEVICE_current_data(&devices[id])[package_num][0] = cur_grp;

    temp = LWPMU_DEVICE_to_read_data(&devices[id])[package_num];
    LWPMU_DEVICE_to_read_data(&devices[id])[package_num] = LWPMU_DEVICE_current_data(&devices[id])[package_num];
    LWPMU_DEVICE_current_data(&devices[id])[package_num] = temp;

    return;
}

/*!
 * @fn       extern   UNC_COMMON_PCI_Read_PMU_Data(param)
 *
 * @param    param    The device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer;
 */
extern VOID
UNC_COMMON_PCI_Read_PMU_Data(
    PVOID           param
)
{
    U32             dev_idx             = *((U32*)param);
    U64             value_low           = 0;
    U64             value_high          = 0;
    U32             this_cpu            = CONTROL_THIS_CPU();
    U64            *buffer              = read_counter_info;
    DRV_CONFIG      pcfg_unc;
    U64             start_index;
    CPU_STATE       pcpu                = &pcb[this_cpu];
    U64             j                   = 0;
    U32             sub_evt_index       = 0;
    S32             prev_ei             = -1;
    S32             cur_ei              = 0;
    U32             cur_grp             = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB             pecb                = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[cur_grp];
    U32             num_events          = 0;
    U32             dev_node            = LWPMU_DEVICE_pci_dev_node_index(&devices[dev_idx]);
    U32             package_num         = 0;
    U32             bus_map_index       = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    if (pecb) {
        num_events = ECB_num_events(pecb);
    }

    package_num         = core_to_package_map[this_cpu];
    bus_map_index       = package_num*MAX_DEVICES + dev_node;
    pcfg_unc            = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index         = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);
    if (unc_package_to_bus_map[bus_map_index] == NULL) {
        return;
    }
    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC(pecb,dev_idx,i) {
        cur_ei = (S32)ECB_entries_group_index(pecb, i);
        //the buffer index for this PMU needs to account for each event
        j = start_index +  ECB_entries_group_index(pecb, i) +
            ECB_entries_emon_event_id_index_local(pecb,i) +
            sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
            package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);

        // read lower 4 bytes
        value_low = PCI_Read(unc_package_to_bus_map[bus_map_index],
                             ECB_entries_dev_no(pecb,i),
                             ECB_entries_func_no(pecb,i),
                             ECB_entries_reg_id(pecb,i));
        value_low &= LOWER_4_BYTES_MASK;

        // read upper 4 bytes
        value_high = PCI_Read(unc_package_to_bus_map[bus_map_index],
                              ECB_entries_dev_no(pecb,i),
                              ECB_entries_func_no(pecb,i),
                              (ECB_entries_reg_id(pecb,i) + NEXT_ADDR_OFFSET));
        buffer[j] = (value_high << NEXT_ADDR_SHIFT) | value_low;
        SEP_PRINT_DEBUG("j = %d value = %llu pkg = %d  e_id = %d\n",j, buffer[j],package_num, ECB_entries_emon_event_id_index_local(pecb,i));
        //Increment sub_evt_index so that the next event position is adjusted
        if ((prev_ei == -1 )|| (prev_ei != cur_ei)) {
             prev_ei = cur_ei;
             sub_evt_index++;
        }
        if (sub_evt_index == num_events) {
            sub_evt_index = 0;
        }
    } END_FOR_EACH_DATA_REG_UNC;

    return;
}

/*!
 * @fn          static VOID UNC_COMMON_PCI_Scan_For_Uncore(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_PCI_Scan_For_Uncore(
    PVOID           param,
    U32             dev_node,
    DEVICE_CALLBACK callback
)
{
    U32                        pci_address;
    U32                        device_id;
    U32                        value;
    U32                        vendor_id;
    U32                        busno;
    U32                        j, k;

    for (busno = 0; busno < 256; busno++) {
         for (j=0; j< MAX_PCI_DEVNO;j++) {
             if (!(UNCORE_TOPOLOGY_INFO_pcidev_valid(&uncore_topology, dev_node, j))) {
                 continue;
             }
             for(k=0;k<MAX_PCI_FUNCNO;k++) {
                 if (!(UNCORE_TOPOLOGY_INFO_pcidev_is_devno_funcno_valid(&uncore_topology,dev_node,j,k))) {
                     continue;
                 }
                 pci_address = FORM_PCI_ADDR(busno,
                                             j,
                                             k,
                                             0);
                 value = PCI_Read_Ulong(pci_address);

                 CHECK_IF_GENUINE_INTEL_DEVICE(value, vendor_id, device_id);

                 SEP_PRINT_DEBUG("device ID = 0x%x\n",device_id);
                 if ( callback && callback->is_Valid_Device && !callback->is_Valid_Device(device_id)) {
                     continue;
                 }
                 UNCORE_TOPOLOGY_INFO_pcidev_is_found_in_platform(&uncore_topology, dev_node, j, k) = 1;
                 SEP_PRINT_DEBUG("found device 0x%x at B:D:F = %d:%d:%d\n", device_id, busno,j,k);
                 UNC_COMMON_Do_Bus_to_Socket_Map(device_id, dev_node, busno);
             }
         }
    }

    return;
}


/************************************************************/
/*
 * UNC common MSR  based API
 *
 ************************************************************/


/*!
 * @fn          extern VOID UNC_COMMON_MSR_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_MSR_Write_PMU (
    PVOID            param,
    U32              control_msr,
    U64              control_val,
    U64              unit_reset_val,
    DEVICE_CALLBACK  callback
)
{
    U32            dev_idx       = *((U32*)param);
    U64            value         = 0;
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];
 
    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    if (control_msr) {
        SYS_Write_MSR(control_msr, control_val);
    }
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        /*
        * Writing the GLOBAL Control register enables the PMU to start counting.
        * So write 0 into the register to prevent any counting from starting.
        */
        if (ECB_entries_reg_id(pecb,i) == control_msr) {
            continue;
        }
        if (unit_reset_val                         &&
            callback                               &&
            callback->is_Unit_Ctl                  &&
            callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {

            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), unit_reset_val);
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
 * @fn         VOID UNC_COMMON_MSR_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
VOID
UNC_COMMON_MSR_Enable_PMU (
    PVOID               param,
    U32                 control_msr,
    U64                 control_value,
    U64                 unit_ctl_value,
    U64                 pmon_ctl_value,
    DEVICE_CALLBACK     callback
)
{
    U32            dev_idx       = *((U32*)param);
    U64            value         = 0;
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (control_msr && (ECB_entries_reg_id(pecb,i) == control_msr)) {
            value = (control_value | ECB_entries_reg_value(pecb,i));
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
            SEP_PRINT_DEBUG("UNC_COMMON_MSR_Write_PMU wrote 0x%x\n", control_msr);
            continue;
        }
        if (callback                               &&
            callback->is_PMON_Ctl                  &&
            callback->is_PMON_Ctl(ECB_entries_reg_id(pecb,i))) {
            value = (pmon_ctl_value | ECB_entries_reg_value(pecb,i));
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
            SEP_PRINT_DEBUG("UNC_COMMON_MSR_Enable_PMU Event_Data_reg = 0x%x --- value 0x%llx\n",
                         ECB_entries_reg_id(pecb,i), value);
            continue;
        }
        if (unit_ctl_value                         &&
            callback                               &&
            callback->is_Unit_Ctl                  &&
            callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {
            value = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
            value &= ~(unit_ctl_value);
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;
    return;
}


/*!
 * @fn         extern VOID UNC_COMMON_MSR_Disable_PMU(PVOID)
 *
 * @brief      Disable the per unit global control to stop the PMU counters.
 *
 * @param      Device Index of this PMU unit
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
extern VOID
UNC_COMMON_MSR_Disable_PMU (
    PVOID               param,
    U32                 control_msr,
    U64                 unit_ctl_value,
    U64                 pmon_ctl_value,
    DEVICE_CALLBACK     callback
)
{
    U32            dev_idx       = *((U32*)param);
    U64            value         = 0;
    U32            this_cpu      = CONTROL_THIS_CPU();
    CPU_STATE      pcpu          = &pcb[this_cpu];

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    if (control_msr) {
        SYS_Write_MSR(control_msr, 0LL);
    }
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (ECB_entries_reg_id(pecb,i) == control_msr) {
            continue;
        }
        if (callback                                &&
            callback->is_Unit_Ctl                   &&
            callback->is_Unit_Ctl(ECB_entries_reg_id(pecb,i))) {
            value = unit_ctl_value | ECB_entries_reg_value(pecb,i);
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
            SEP_PRINT_DEBUG("UNC_COMMON_MSR_Disable_PMU Event_Data_reg = 0x%x --- value 0x%llx\n",
                         ECB_entries_reg_id(pecb,i), value);
            continue;
        }
        if (pmon_ctl_value                          &&
            callback                                &&
            callback->is_PMON_Ctl                   &&
            callback->is_PMON_Ctl(ECB_entries_reg_id(pecb,i))) {
            value = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
            value &= ~(pmon_ctl_value);
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), value);
            SEP_PRINT_DEBUG("UNC_COMMON_MSR_Disable_PMU Event_Data_reg = 0x%x --- value 0x%llx\n",
                         ECB_entries_reg_id(pecb,i), value);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;
    return;
}


/*!
 * @fn UNC_COMMON_MSR_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 */
VOID
UNC_COMMON_MSR_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64  *data       = (U64*) param;
    U32   cur_grp    = LWPMU_DEVICE_cur_group(&devices[id]);
    ECB   pecb       = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    U32   this_cpu   = CONTROL_THIS_CPU();
    U64   value      = 0;
    U32   index      = 0;
    U64   diff       = 0;

    // Write GroupID
    data    = (U64*)((S8*)data + ECB_group_offset(pecb));
    *data   = cur_grp + 1;

    FOR_EACH_DATA_REG_UNC(pecb, id, i) {
        data  = (U64 *)((S8*)param + ECB_entries_counter_event_offset(pecb,i));
        value = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
        //check for overflow
        if (value < LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index]) {
            diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index];
            diff += value;
        }
        else {
            diff = value - LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index];
        }
        LWPMU_DEVICE_acc_per_thread(&devices[id])[this_cpu][index] += diff;
        LWPMU_DEVICE_prev_val_per_thread(&devices[id])[this_cpu][index] = value;
        *data = LWPMU_DEVICE_acc_per_thread(&devices[id])[this_cpu][index];
        index++;
    } END_FOR_EACH_DATA_REG_UNC;

    return;
}


/* ------------------------------------------------------------------------- */
/*!
 * @fn       void UNC_COMMON_MSR_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
extern  VOID
UNC_COMMON_MSR_Trigger_Read (
    U32    id
)
{
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = core_to_package_map[this_cpu];
    U32             cur_grp;
    ECB             pecb;
    CPU_STATE       pcpu                = &pcb[this_cpu];
    U32             index               = 0;
    U64             diff                = 0;
    U64             value;
    U64            *temp;

    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_UNINITIALIZED ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_IDLE          ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_RESERVED      ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_PREPARE_STOP  ||
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_STOPPED) {
        return;
    }

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }

    cur_grp = LWPMU_DEVICE_cur_group(&devices[id]);
    pecb    = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];
    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC_VER2(pecb, id, i) {
        value = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
        //check for overflow
        if (value < LWPMU_DEVICE_prev_value(&devices[id])[package_num][index]) {
            diff = LWPMU_DEVICE_counter_mask(&devices[id]) - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
            diff += value;
        }
        else {
            diff = value - LWPMU_DEVICE_prev_value(&devices[id])[package_num][index];
        }
        LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index] += diff;
        LWPMU_DEVICE_prev_value(&devices[id])[package_num][index] = value;
        LWPMU_DEVICE_current_data(&devices[id])[package_num][index+1] = LWPMU_DEVICE_acc_value(&devices[id])[package_num][cur_grp][index];
        index++;
    } END_FOR_EACH_DATA_REG_UNC_VER2;
    LWPMU_DEVICE_current_data(&devices[id])[package_num][0] = cur_grp;

    temp = LWPMU_DEVICE_to_read_data(&devices[id])[package_num];
    LWPMU_DEVICE_to_read_data(&devices[id])[package_num] = LWPMU_DEVICE_current_data(&devices[id])[package_num];
    LWPMU_DEVICE_current_data(&devices[id])[package_num] = temp;

    return;
}


/*!
 * @fn UNC_COMMON_MSR_Read_PMU_Data(param)
 *
 * @param    param    The read thread node to process
 * @param    id       The id refers to the device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer
 *           Let us say we have 2 core events in a dual socket JKTN;
 *           The start_index will be at 32 as it will 2 events in 16 CPU per socket
 *           The position for first event of QPI will be computed based on its event
 *
 */
VOID
UNC_COMMON_MSR_Read_PMU_Data (
    PVOID  param
)
{
    U32             dev_idx             = *((U32*)param);
    U32             this_cpu            = CONTROL_THIS_CPU();
    U32             package_num         = 0;
    U64            *buffer              = read_counter_info;
    DRV_CONFIG      pcfg_unc;
    U64             start_index;
    CPU_STATE       pcpu                = &pcb[this_cpu];
    U64             j                   = 0;
    U32             sub_evt_index       = 0;
    U32             prev_ei             = -1;
    U32             cur_ei              = 0;
    U32             cur_grp             = LWPMU_DEVICE_cur_group(&devices[(dev_idx)]);
    ECB             pecb                = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[cur_grp];
    U32             num_events          = 0;

    if (!CPU_STATE_socket_master(pcpu)) {
        return;
    }
    if (pecb) {
        num_events = ECB_num_events(pecb);
    }
    package_num         = core_to_package_map[this_cpu];
    pcfg_unc            = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
    start_index         = DRV_CONFIG_emon_unc_offset(pcfg_unc, cur_grp);
    SEP_PRINT_DEBUG("offset for uncore group %d is %d num_pkgs = 0x%llx num_events = %d\n", cur_grp, start_index, num_packages, num_events);
    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC(pecb,dev_idx,i) {
            cur_ei = ECB_entries_group_index(pecb, i);
            //the buffer index for this PMU needs to account for each event
            j = start_index +  ECB_entries_group_index(pecb, i) +
                ECB_entries_emon_event_id_index_local(pecb,i) +
                sub_evt_index*num_packages*LWPMU_DEVICE_num_units(&devices[dev_idx])+
                package_num * LWPMU_DEVICE_num_units(&devices[dev_idx]);
                SEP_PRINT_DEBUG("%d + %d + %d + %d*%d*%d + %d * %d = j \n",
                      start_index,ECB_entries_group_index(pecb, i),ECB_entries_emon_event_id_index_local(pecb,i),
                      sub_evt_index,num_packages,LWPMU_DEVICE_num_units(&devices[dev_idx]), package_num,LWPMU_DEVICE_num_units(&devices[dev_idx]));
            buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
            SEP_PRINT_DEBUG("j = %d value = 0x%x pkg = %d  e_id = %d\n",j, buffer[j], package_num, ECB_entries_emon_event_id_index_local(pecb,i));
            //Increment sub_evt_index so that the next event position is adjusted
            if ((prev_ei == -1 )|| (prev_ei != cur_ei)) {
                 prev_ei = cur_ei;
                 sub_evt_index++;
            }
            if (sub_evt_index == num_events) {
                sub_evt_index = 0;
            }
    } END_FOR_EACH_DATA_REG_UNC;

    return;
}


/*!
 * @fn         VOID UNC_COMMON_MSR_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
VOID
UNC_COMMON_MSR_Clean_Up (
    VOID   *param
)
{
    U32 dev_idx = *((U32*)param);
 
    FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i) {
        if (ECB_entries_clean_up_get(pecb,i)) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb,i), 0LL);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

