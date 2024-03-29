/*COPYRIGHT**
    Copyright (C) 2002-2016 Intel Corporation.  All Rights Reserved.

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

#ifndef __ERROR_REPORTING_UTILS_H__
#define __ERROR_REPORTING_UTILS_H__

#define DRV_ASSERT_N_RET_VAL(ret_val)                                    \
    DRV_ASSERT((ret_val) == VT_SUCCESS);                                 \
    DRV_CHECK_N_RETURN_N_FAIL(ret_val);

#define DRV_ASSERT_N_CONTINUE(ret_val)                                   \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
    }

#define DRV_CHECK_N_RETURN_N_FAIL(ret_val)                               \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
        return (ret_val);                                                \
    }

#define DRV_CHECK_N_RETURN_NO_RETVAL(ret_val)                            \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
        return;                                                          \
    }

#define DRV_CHECK_PTR_N_RET_VAL(ptr)                                     \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_CHECK_PTR_N_RET_NULL(ptr)                                    \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        return NULL;                                                     \
    }

#define DRV_CHECK_PTR_N_LOG_NO_RETURN(ptr)                               \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
    }

#define DRV_CHECK_N_LOG_NO_RETURN(ret_val)                               \
    if ((ret_val) != VT_SUCCESS) {                                       \
        LOG_ERR1(VTSA_T("Operation failed with error code "),(ret_val)); \
    }
    
#define DRV_CHECK_N_RET_NEG_ONE(ret_val)                                 \
    if ((ret_val) == -1) {                                               \
        LOG_ERR0(VTSA_T("Operation failed with error code = -1"));       \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_REQUIRES_TRUE_COND_RET_N_FAIL( cond )                        \
    if ( !(cond) ) {                                                     \
        LOG_ERR0(VTSA_T("Condition check failed"));                      \
        return VT_SAM_ERROR;                                             \
    }

#define DRV_REQUIRES_TRUE_COND_RET_ASSIGNED_VAL( cond, ret_val)         \
    if ( !(cond) ) {                                                    \
        LOG_ERR0(VTSA_T("Condition check failed"));                     \
        return ret_val;                                                 \
    }

#define DRV_CHECK_N_ERR_LOG_ERR_STRNG_N_RET( rise_err )                \
    if (rise_err != VT_SUCCESS) {                                      \
        PVOID rise_ptr = NULL;                                         \
        const VTSA_CHAR *error_str = NULL;                             \
        RISE_open(&rise_ptr);                                          \
        RISE_translate_err_code(rise_ptr, rise_err, &error_str);       \
        LogItW(LOG_LEVEL_ERROR|LOG_AREA_GENERAL, L"Operation failed with error [ %d ] = %s\n",rise_err,error_str); \
        RISE_close(rise_ptr);                                          \
        return rise_err;                                               \
    }

#define DRV_CHECK_PTR_N_CLEANUP(ptr, gotolabel, ret_val)                 \
    if ((ptr) == NULL) {                                                 \
        LOG_ERR0(VTSA_T("Encountered null pointer"));                    \
        ret_val = VT_SAM_ERROR;                                          \
        goto gotolabel;                                                  \
    }

#define DRV_CHECK_ON_FAIL_CLEANUP_N_RETURN(ret_val, gotolabel)         \
    if ((ret_val) != VT_SUCCESS) {                                     \
        DRV_CHECK_N_LOG_NO_RETURN(ret_val);                            \
        goto gotolabel;                                                \
    } 


#define DRV_CHECK_N_CLEANUP_N_RETURN_RET_NEG_ONE(ret_val, gotolabel)   \
    if ((ret_val) == -1) {                                             \
        DRV_CHECK_N_LOG_NO_RETURN(ret_val);                            \
        goto gotolabel;                                                \
    } 

#define DRV_CHECK_PTR_ON_NULL_CLEANUP_N_RETURN(ptr, gotolabel)         \
    if ((ptr) == NULL) {                                               \
        DRV_CHECK_PTR_N_LOG_NO_RETURN(ptr);                            \
        goto gotolabel;                                                \
    } 
    
#define FREE_N_SET_NULL(ptr)                                           \
    if (ptr != NULL) {                                                 \
        free(ptr);                                                     \
        ptr = NULL;                                                    \
    }

#define DELETE_N_SET_NULL(ptr)                                         \
        delete ptr;                                                    \
        ptr = NULL;

#endif

