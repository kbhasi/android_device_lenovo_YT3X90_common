/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2015 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef __SW_STRUCTS_H__
#define __SW_STRUCTS_H__ 1

#include "sw_types.h"

/*
 * Convenience for a 'string' data type.
 * Not strictly required.
 */
typedef struct sw_string_type sw_string_type_t;
#pragma pack(push, 1)
struct sw_string_type {
    pw_u16_t len;
    char data[1];
};
#pragma pack(pop)
#define SW_STRING_TYPE_HEADER_SIZE() (sizeof(struct sw_string_type) - sizeof(char[1]))

typedef struct sw_file_payload sw_file_payload_t;
#pragma pack(push, 1)
struct sw_file_payload {
    pw_u16_t m_numPathValuePairs;
    char data[1];
};
#pragma pack(pop)
#define SW_FILE_PAYLOAD_HEADER_SIZE() (sizeof(struct sw_file_payload) - sizeof(char[1]))

/*
 * Struct used to encode CONSTANT payloads.
 */
typedef struct sw_file_payload sw_const_payload;

/*
 * MSR specifiers
 */
typedef struct sw_msr_identifier sw_msr_identifier_t;
#pragma pack(push, 1)
struct sw_msr_identifier {
    u8 subtype:4; // For "PC6" v "PC6C" differentiation etc.
    u8 type:4; // One of 'sw_msr_type_t'
    u8 depth; // Actual MSR number e.g. "CC2" will have a "2" here
};
#pragma pack(pop)
/*
 * Struct to encode MSR addresses.
 */
typedef struct sw_msr_addr sw_msr_addr_t;
#pragma pack(push, 1)
struct sw_msr_addr {
    sw_msr_identifier_t id; // MSR identifier
    u32 addr; // MSR address
};
#pragma pack(pop)
/*
 * Struct to encode MSR values.
 */
typedef struct sw_msr_val sw_msr_val_t;
struct sw_msr_val {
    sw_msr_identifier_t id; // MSR identifier
    u64 val; // MSR value
};
/*
 * Structure used by Ring-3 to tell the power driver which
 * MSRs to read.
 */
typedef struct sw_msr_info sw_msr_info_t;
struct sw_msr_info {
    u16 num_msr_addrs; // The number of MSR addresses pointed to by the 'data' field
    char data[1]; // The list of 'sw_msr_addr_t' instances to read.
};
#define SW_MSR_INFO_HEADER_SIZE() ( sizeof(sw_msr_info_t) - sizeof(char[1]) )

typedef enum sw_kernel_wakelock_type {
    SW_WAKE_LOCK=0, // A kernel wakelock was acquired
    SW_WAKE_UNLOCK=1, // A kernel wakelock was released
    SW_WAKE_LOCK_TIMEOUT=2, // A kernel wakelock was acquired with a timeout
    SW_WAKE_LOCK_INITIAL=3, // A kernel wakelock was acquired before the collection started
    SW_WAKE_UNLOCK_ALL=4, // All previously held kernel wakelocks were released -- used in ACPI S3 notifications
} sw_kernel_wakelock_type_t;

typedef enum sw_when_type {
    SW_WHEN_TYPE_BEGIN=0, /* Start snapshot */
    SW_WHEN_TYPE_POLL,
    SW_WHEN_TYPE_NOTIFIER,
    SW_WHEN_TYPE_TRACEPOINT,
    SW_WHEN_TYPE_END, /* Stop snapshot */
    SW_WHEN_TYPE_NONE
} sw_when_type_t;

#define SW_READ_TRIGGER_BEGIN_MASK() (1U << SW_WHEN_TYPE_BEGIN)
#define SW_READ_TRIGGER_END_MASK() (1U << SW_WHEN_TYPE_END)
#define SW_READ_TRIGGER_POLL_MASK() (1U << SW_WHEN_TYPE_POLL)
#define SW_READ_TRIGGER_TRACEPOINT_MASK() (1U << SW_WHEN_TYPE_TRACEPOINT)
#define SW_READ_TRIGGER_NOTIFIER_MASK() (1U << SW_WHEN_TYPE_NOTIFIER)
#define SW_GET_READ_TRIGGER_MASK_VALUE(m) (1U << (m))

typedef enum sw_io_cmd {
    SW_IO_CMD_READ=0,
    SW_IO_CMD_WRITE,
    SW_IO_CMD_MAX
} sw_io_cmd_t;

typedef struct sw_driver_msr_io_descriptor sw_driver_msr_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_msr_io_descriptor {
    pw_u64_t address;
};
#pragma pack(pop)

#if 0
typedef struct sw_driver_ipc_io_descriptor sw_driver_ipc_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_ipc_io_descriptor {
    // TODO
};
#pragma pack(pop)

typedef struct sw_driver_mmio_io_descriptor sw_driver_mmio_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_mmio_io_descriptor {
    // TODO
};
#pragma pack(pop)
#endif // if 0

typedef struct sw_driver_ipc_mmio_io_descriptor sw_driver_ipc_mmio_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_ipc_mmio_io_descriptor {
    union {
        struct {
            pw_u16_t command;
            pw_u16_t sub_command;
        };
        pw_u32_t ipc_command; // (sub_command << 12) | (command)
    };
    // TODO: add a section for 'ctrl_address' and 'ctrl_remapped_address'
    union {
        pw_u64_t data_address; // Will be "io_remapped"
        pw_u64_t data_remapped_address;
    };
};
#pragma pack(pop)

typedef struct sw_driver_pci_io_descriptor sw_driver_pci_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_pci_io_descriptor {
    pw_u32_t bus;
    pw_u32_t device;
    pw_u32_t function;
    pw_u32_t offset;
};
#pragma pack(pop)

typedef struct sw_driver_configdb_io_descriptor sw_driver_configdb_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_configdb_io_descriptor {
    // pw_u32_t port;
    // pw_u32_t offset;
    pw_u32_t address;
};
#pragma pack(pop)

typedef struct sw_driver_trace_args_io_descriptor sw_driver_trace_args_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_trace_args_io_descriptor {
    pw_u8_t num_args; // Number of valid entries in the 'args' array, below; 1 <= num_args <= 7
    pw_u8_t args[7]; // Max of 7 args can be recorded
};
#pragma pack(pop)

typedef struct sw_driver_io_descriptor sw_driver_io_descriptor_t;
#pragma pack(push, 1)
struct sw_driver_io_descriptor {
    pw_u16_t collection_type;
    // TODO: specify READ/WRITE
    pw_s16_t collection_command; // One of 'sw_io_cmd_t'
    pw_u16_t counter_size_in_bytes; // The number of bytes to READ
    union {
        struct sw_driver_msr_io_descriptor msr_descriptor;
        // struct sw_driver_ipc_io_descriptor ipc_descriptor;
        // struct sw_driver_mmio_io_descriptor mmio_descriptor;
        struct sw_driver_ipc_mmio_io_descriptor ipc_descriptor;
        struct sw_driver_ipc_mmio_io_descriptor mmio_descriptor;
        struct sw_driver_pci_io_descriptor pci_descriptor;
        struct sw_driver_configdb_io_descriptor configdb_descriptor;
        struct sw_driver_trace_args_io_descriptor trace_args_descriptor;
    };
    pw_u64_t write_value; // The value to WRITE
};
#pragma pack(pop)

typedef struct sw_driver_interface_info sw_driver_interface_info_t;
#pragma pack(push, 1)
struct sw_driver_interface_info {
    pw_u64_t tracepoint_id_mask;
    pw_u64_t notifier_id_mask;
    pw_s16_t cpu_mask; // Which CPU(s) should the driver read the specified address on?
                       // Currently: -2 ==> read on ALL CPUs, -1 ==> read on ANY CPU, >= 0 ==> the specific CPU to read on
    pw_s16_t plugin_id; // Metric Plugin SID
    pw_s16_t metric_id; // Metric ID -- domain specific ID assigned by each Metric Plugin
    pw_s16_t msg_id; // Message ID -- ID retrieved from the SoC Watch config file
    // enum sw_io_type collection_type;
    pw_u16_t num_io_descriptors; // Number of IO descriptors in the 'descriptors' array, below
    // pw_u16_t total_counter_size_in_bytes;
    pw_u8_t read_trigger_bits;
    // pw_u8_t counter_size_in_bytes; // Usually either 4 (for 32b counters) or 8 (for 64b counters)
    pw_u8_t descriptors[1];
};
#pragma pack(pop)

#define SW_DRIVER_INTERFACE_INFO_HEADER_SIZE() (sizeof(struct sw_driver_interface_info) - sizeof(pw_u8_t[1]))

typedef struct sw_driver_interface_msg sw_driver_interface_msg_t;
#pragma pack(push, 1)
struct sw_driver_interface_msg {
    pw_u16_t num_infos; // Number of 'sw_driver_interface_info' structs contained within the 'infos' variable, below
    // pw_u16_t infos_size_bytes; // Size of data inlined within the 'infos' variable, below
    pw_u8_t infos[1];
};
#pragma pack(pop)
#define SW_DRIVER_INTERFACE_MSG_HEADER_SIZE() (sizeof(struct sw_driver_interface_msg) - sizeof(pw_u8_t[1]))

typedef enum sw_name_id_type {
    SW_NAME_TYPE_TRACEPOINT,
    SW_NAME_TYPE_NOTIFIER,
    SW_NAME_TYPE_COLLECTOR,
    SW_NAME_TYPE_MAX,
} sw_name_id_type_t;

typedef struct sw_name_id_pair sw_name_id_pair_t;
#pragma pack(push, 1)
struct sw_name_id_pair {
    pw_u16_t id;
    pw_u16_t type; // One of 'sw_name_id_type'
    struct sw_string_type name;
};
#pragma pack(pop)
#define SW_NAME_ID_HEADER_SIZE() (sizeof(struct sw_name_id_pair) - sizeof(struct sw_string_type))

typedef struct sw_name_info_msg sw_name_info_msg_t;
#pragma pack(push, 1)
struct sw_name_info_msg {
    pw_u16_t num_name_id_pairs;
    pw_u16_t payload_len;
    pw_u8_t pairs[1];
};
#pragma pack(pop)

typedef struct sw_driver_msg sw_driver_msg_t;
#pragma pack(push, 1)
struct sw_driver_msg {
    pw_u64_t tsc;
    pw_u16_t cpuidx;
    pw_u8_t plugin_id; // Cannot have more than 256 plugins?
    pw_u8_t metric_id; // Each plugin cannot handle more than 256 metrics?
    pw_u8_t msg_id; // Each metric cannot have more than 256 components?
    pw_u16_t payload_len;
    // pw_u64_t p_payload; // Ptr to payload
    union {
        pw_u64_t __dummy; // Ensure size of struct is consistent on x86, x64
        char *p_payload; // Ptr to payload
    };
};
#pragma pack(pop)
#define SW_DRIVER_MSG_HEADER_SIZE() (sizeof(struct sw_driver_msg) - sizeof(pw_u64_t))

typedef enum sw_driver_collection_cmd {
    SW_DRIVER_START_COLLECTION=1,
    SW_DRIVER_STOP_COLLECTION=2,
    SW_DRIVER_PAUSE_COLLECTION=3,
    SW_DRIVER_RESUME_COLLECTION=4,
    SW_DRIVER_CANCEL_COLLECTION=5,
} sw_driver_collection_cmd_t;

typedef struct sw_driver_version_info sw_driver_version_info_t;
#pragma pack(push, 1)
struct sw_driver_version_info {
    pw_u16_t major;
    pw_u16_t minor;
    pw_u16_t other;
};
#pragma pack(pop)

/*
 * Wrapper for ioctl arguments.
 * EVERY ioctl MUST use this struct!
 */
typedef struct sw_driver_ioctl_arg sw_driver_ioctl_arg_t;
#pragma pack(push, 1)
struct sw_driver_ioctl_arg {
    pw_s32_t in_len;
    pw_s32_t out_len;
    // pw_u64_t p_in_arg; // Pointer to input arg
    // pw_u64_t p_out_arg; // Pointer to output arg
    char *in_arg;
    char *out_arg;
};
#pragma pack(pop)

#endif // __SW_STRUCTS_H__
