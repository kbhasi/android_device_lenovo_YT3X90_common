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

#ifndef _PW_USER_DEFINES_H_
#define _PW_USER_DEFINES_H_ 1

#ifndef __KERNEL__

#include <stdio.h> // for "FILE *"
#include <errno.h> // for "PW_PERROR"
#include <string.h> // for "PW_PERROR"
#ifndef _WIN32
    #include <sys/time.h> // for "gettimeofday"
#endif // windows
#include <fstream> // for "std::ofstream"
#include <errno.h> // for "perror"
#include <string.h> // for "strerror"
#include "sw_types.h"

/* ***************************************************
 * The following is valid only for userspace code.
 * ***************************************************
 */
/*
 * Default output file name -- the extensions depend on
 * which program is executing: wuwatch output files have
 * a ".sw1" extension, while wudump output files have a
 * ".txt" extension. The extensions are added in by the
 * respective programs i.e. wuwatch/wudump.
 */
#define DEFAULT_WUWATCH_OUTPUT_FILE_NAME "wuwatch_output"
/*
 * Default wuwatch config file name.
 */
#define DEFAULT_WUWATCH_CONFIG_FILE_NAME "wuwatch_config.txt"
/*
 * Default SWA 2.0 output file prefix. Actual file will
 * be called 'SoCWatchOutput.sw2'
 */
#define DEFAULT_SWA_OUTPUT_FILE_PREFIX "SoCWatchOutput"
/*
 * Default SWA 2.0 config file name.
 */
#define DEFAULT_SWA_CONFIG_FILE_NAME "SOCWatchConfig.txt"
/*
 * Default SWA 2.0 config file path.
 */
#define DEFAULT_SWA_CONFIG_FILE_PATH "./SOCWatchConfig.txt"
/*
 * Default SWA 2.0 polling interval.
 */
#define DEFAULT_SWA_POLLING_INTERVAL_MSECS 100
/*
 * Macro to convert a {major.minor.other} version into a
 * single 32-bit unsigned version number.
 * This is useful when comparing versions, for example.
 * Pretty much identical to the 'KERNEL_VERSION(...)' macro.
 */
//#define WUWATCH_VERSION(major, minor, other) ( (2^16) * (major) + (2^8) * (minor) + (other) )
// #define COLLECTOR_VERSION(major, minor, other) ( (2^16) * (major) + (2^8) * (minor) + (other) )
#define COLLECTOR_VERSION(major, minor, other) ( ((major) << 16) + ((minor) << 8) + (other) )

#define PW_SWA_RESULT_FILE_MAGIC_HEADER (0x8000000000000000ULL)
#define PW_EXTRACT_SWA_MAGIC_HEADER(m) ( (m) & PW_SWA_RESULT_FILE_MAGIC_HEADER )
#define PW_EXTRACT_SWA_VERSION_NUMBER(m) (pw_u32_t)( (m) & ~PW_SWA_RESULT_FILE_MAGIC_HEADER )
#define PW_GET_MAJOR_FROM_COLLECTOR(v) (pw_u8_t) ( (v) >> 16 & 0xff )
#define PW_GET_MINOR_FROM_COLLECTOR(v) (pw_u8_t) ( (v) >> 8 & 0xff )
#define PW_GET_OTHER_FROM_COLLECTOR(v) (pw_u8_t) ( (v) & 0xff )
#define PW_CONVERT_COLLECTOR_VERSION_TO_STRING(v) ({std::stringstream __stream; \
        __stream << (int)PW_GET_MAJOR_FROM_COLLECTOR(v) << "." << (int)PW_GET_MINOR_FROM_COLLECTOR(v) \
        << "." << (int)PW_GET_OTHER_FROM_COLLECTOR(v); __stream.str();})
/*
 * Stringify.
 */
#define __STRING(x) #x
#define TO_STRING(x) __STRING(x)
/*
 * Iterate over vectors, deques and ranges.
 */
#define for_each_ptr_in_vector(ptr, vector) for (size_t __curr=0, __end=(vector).size(); __curr!=__end && (ptr=(vector)[__curr]); ++__curr)
#define for_each_ptr_in_range(ptr, begin, end) for (typeof(begin) __curr=(begin); __curr!=(end) && (ptr=*__curr); ++__curr)
/*
 * Find a string 'str' in an array of 'std::string' instances
 */
#define FIND_STRING_IN_ARRAY(ptr, str, array) ({int s = SW_ARRAY_SIZE(array); bool found = (ptr = std::find((array), (array)+s, (str))) != &((array)[s]); found;})

/* **************************************
 * Debugging tools.
 * **************************************
 */
enum pw_log_level_t {
    PW_LOG_LEVEL_FATAL=0, /* Print FATALs */
    PW_LOG_LEVEL_ERROR=1, /* Print ERRORs */
    PW_LOG_LEVEL_WARNING=2, /* Print ERRORs + WARNINGs */
    PW_LOG_LEVEL_DEBUG=3, /* Print ERRORs + WARNINGs + DEBUGs */
    PW_LOG_LEVEL_INFO=4 /* Print ERRORs + WARNINGs + DEBUGs + INFORMATIONALs */
};

/* Controls the verbosity of output. Levels are:
 *  - 0: only fatal message outputs
 *  - 1: errors
 *  - 2: errors + warnings
 *  - 3: errors + warnings + debug
 *  - 4: errors + warnings + debug + informational statements
 *
 *  *************************************************************
 *  Set to 'PW_LOG_LEVEL_INFO' (i.e. most verbose), for now. Reset to
 *  'PW_LOG_LEVEL_ERROR' before release!!!
 *  *************************************************************
 */
extern pw_u16_t g_verbosity;

extern FILE *s_errorLogFP;
extern FILE *s_debugLogFP;
extern std::ofstream g_nullStream;

//Defines to make the macros below work
// #define PW_LOG_LEVEL_FATAL PW_LOG_LEVEL_ERROR
#define s_FATALLogFP s_errorLogFP
#define s_ERRORLogFP s_errorLogFP
#define s_INFOLogFP s_debugLogFP
#define s_WARNINGLogFP s_debugLogFP
#define s_DEBUGLogFP s_debugLogFP
#define PW_FATAL_STREAM std::cerr
#define PW_ERROR_STREAM std::cerr
#define PW_WARNING_STREAM std::cerr
#define PW_INFO_STREAM std::cout
#define PW_DEBUG_STREAM std::cout

#define PW_LOG_OUTPUT(level, fp, format, ...) do { if (unlikely(g_verbosity && (level) <= g_verbosity)){ fprintf(fp, format, ##__VA_ARGS__); fflush(fp);}} while(0);
#define PW_GET_STREAM_HELPER(level, stream) ( (g_verbosity && (level) <= g_verbosity) ? (stream) : g_nullStream)
/*
 * Helper macros to print information
 */
#define PW_LOG_FATAL(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_FATAL, s_errorLogFP, "FATAL:   " format, ##__VA_ARGS__);
#define PW_LOG_ERROR(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_ERROR, s_errorLogFP, "ERROR:   " format, ##__VA_ARGS__);
#define PW_LOG_FATAL_LINE(format, ...) \
    do { \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_FATAL, s_errorLogFP, "FATAL:   " format, ##__VA_ARGS__); \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, s_errorLogFP, "         at %s:%d\n", __FILE__, __LINE__);\
    } while (0);
#define PW_LOG_ERROR_LINE(format, ...) \
    do { \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_ERROR, s_errorLogFP, "ERROR:   " format, ##__VA_ARGS__); \
        PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, s_errorLogFP, "         at %s:%d\n", __FILE__, __LINE__);\
    } while (0);
#define PW_LOG_WARNING(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_WARNING, s_debugLogFP, "WARNING: " format, ##__VA_ARGS__)
#define PW_LOG_DEBUG(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_DEBUG, s_debugLogFP,     "DEBUG:   " format, ##__VA_ARGS__)
#define PW_LOG_INFO(format, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_INFO, s_debugLogFP,       "INFO:    " format, ##__VA_ARGS__)
#define PW_LOG(type, ...) PW_LOG_OUTPUT(PW_LOG_LEVEL_##type, s_##type##LogFP, __VA_ARGS__)
#define PW_LOG_FORCE(format, ...) fprintf(s_debugLogFP, format, ##__VA_ARGS__) /* Force a debug printf */

#define PW_GET_FATAL_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_FATAL, std::cerr << "FATAL:   ")
#define PW_GET_ERROR_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_ERROR, std::cerr << "ERROR:   ")
#define PW_GET_WARNING_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_WARNING, std::cerr << "WARNING: ")
#define PW_GET_DEBUG_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_DEBUG, std::cout << "DEBUG:   ")
#define PW_GET_INFO_STREAM() PW_GET_STREAM_HELPER(PW_LOG_LEVEL_INFO, std::cout << "INFO:    ")
#define PW_GET_STREAM(type) PW_GET_STREAM_HELPER(PW_LOG_LEVEL_##type, PW_##type##_STREAM)
#define PW_GET_FORCE_STREAM() (std::cerr)

/*
 * Macros to copy and or assert.
 */
#define PW_DEBUG_COPY(level, ...) do { \
    if (unlikely(g_verbosity && (level) <= g_verbosity)) { \
        std::copy(__VA_ARGS__); \
    } \
} while (0)
#define PW_DEBUG_ASSERT(level, cond, ...) do { \
    if (unlikely(g_verbosity && (level) <= g_verbosity && !(cond))) { \
        PW_LOG_ERROR(__VA_ARGS__); \
        assert(false); \
    } \
} while (0)


/*
 * Macros to trace function enters and exits.
 */
#if DEVELOPMENT_MODE // Development code; NOT meant for production
    #define PW_TRACE_FUNCTION_ENTER() do { \
        PW_LOG_INFO("Entering function %s\n", __FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_EXIT() do { \
        PW_LOG_INFO("Exiting function %s\n", __FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_ENTER_VERBOSE() do { \
        PW_LOG_INFO("Entering function %s\n", __PRETTY_FUNCTION__); \
    } while(0)

    #define PW_TRACE_FUNCTION_EXIT_VERBOSE() do { \
        PW_LOG_INFO("Exiting function %s\n", __PRETTY_FUNCTION__); \
    } while(0)
    /*
     * Basic timer-based profiling functions.
     * Every 'ENTER' MUST be accompanied by
     * a corresponding 'EXIT'!
     */
    #define PW_TIME_FUNCTION_ENTER() { \
        PW_LOG_INFO("Entering function %s\n", __PRETTY_FUNCTION__); \
        pwr::Timer __timer(__FUNCTION__);

    #define PW_TIME_FUNCTION_EXIT() \
        PW_LOG_INFO("Exiting function %s\n", __PRETTY_FUNCTION__); \
    }
#else // Production code
    #define PW_TRACE_FUNCTION_ENTER() /* NOP */

    #define PW_TRACE_FUNCTION_EXIT() /* NOP */

    #define PW_TRACE_FUNCTION_ENTER_VERBOSE() /* NOP */

    #define PW_TRACE_FUNCTION_EXIT_VERBOSE() /* NOP */

    #define PW_TIME_FUNCTION_ENTER() { /* NOP */

    #define PW_TIME_FUNCTION_EXIT() } /* NOP */
#endif // DEVELOPMENT_MODE

#define PW_DO_REPORT_FILE_ERROR(msg, path) do { \
    PW_LOG_ERROR(msg, (path).c_str(), strerror(errno)); \
} while(0)

#define PW_TODO_MSG(msg) do { \
    PW_LOG_ERROR("%s functionality is TODO!\n", (msg)); \
} while(0)

#define PW_TODO() do { \
    PW_TODO_MSG(__PRETTY_FUNCTION__); \
} while(0)

#define PW_PERROR_LEVEL(msg, level) do { \
    PW_GET_STREAM(level) << msg << ": " << strerror(errno) << std::endl; \
} while(0)

#define PW_PERROR(msg) PW_PERROR_LEVEL(msg, ERROR)

/*
 * A convenience macro to return the number
 * of micro seconds elapsed since the epoch.
 */
#define PW_GET_CURR_TIME_USECS() ({ \
        double __time=0.0; \
        struct timeval __tv; \
        if (gettimeofday(&__tv, NULL) == 0) { \
            __time = __tv.tv_sec * 1e6 + __tv.tv_usec; \
        } \
        __time;})


/*
 * Macros corresponding to the kernel versions of 'likely()'
 * and 'unlikely()' -- GCC SPECIFIC ONLY!
 */
#if defined (__linux__)
	#define likely(x) __builtin_expect(!!(x), 1)
	#define unlikely(x) __builtin_expect(!!(x), 0)
#else // windows
	#define likely(x) (!!(x))
	#define unlikely(x) (!!(x))

	#define __attribute__(a) // ignore __attribute__ macros on Windows
#endif // linux

/* Nanoseconds in a second */
#define NANOSEC_PER_SEC (1000000000ULL)
#define MILLISEC_PER_SEC (1000ULL)
#define SEC_PER_MILLISEC ((double)1.0/MILLISEC_PER_SEC)
#ifndef USEC_PER_SEC /* avoid redefinition in driver build */
#define USEC_PER_SEC (1000000ULL)
#endif
#define SEC_PER_USEC ((double)1.0/USEC_PER_SEC)

#endif // __KERNEL__

#endif // _PW_USER_DEFINES_H_
