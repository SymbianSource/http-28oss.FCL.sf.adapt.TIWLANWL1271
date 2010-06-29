/*
 * gendebug.h
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.      
 * All rights reserved.      
 * 
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License v1.0 or BSD License which accompanies
 * this distribution. The Eclipse Public License is available at
 * http://www.eclipse.org/legal/epl-v10.html and the BSD License is as below.                                   
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
* ============================================================================
*  Name     : gendebug from gendebug.h
*  Part of  : WLAN Networking

* ============================================================================
*/


#ifndef GENDEBUG_H
#define GENDEBUG_H

#ifndef __KERNEL_MODE__
#include <e32svr.h>
#endif

/** @file gendebug.h
    @brief Common helper file for debugging. 

    The file contains methods for adding traces and hardcoded breakpoint.

    @page page1 A documentation for general helper module for doing debugging and tracing.

    @subsection subsec1 This subsection describes the usage of hardcoded breakpoints.

    Hardcoded breakpoints are usefull for allowing the execution to stop in right places
    without bringing the OS down like using general assertions would do. Hardcoded breakpoints
    must be used together with debugger. The hardcoded breakpoints are used by inserting
    macro _DBG_MUST_BREAK into source code.

    @note This feature has not been implemented yet.

    @subsection subsec2 This subsection describes the usage of trace utilities.

    Tracing is the most important way of doing HW level debugging (after looking the source of course).
    Most of times users don't have access to debuggers and doing debugging in the field can be greatly
    improved if there are great variety of configurable traces available.

    In order to use the macro in kernel space, ALWAYS_KERNEL must be defined in the MMP-file of the module.

    To use static tracing the module using these utilities must have defined DEBUG_LEVEL (this is integer constant) in
    MMP-file. This constant defines the bit-mask for the set of traces, which are put into code at the compile time. See
    the macro definitions for details.

    The other way to do tracing is to use run-time tracing, which requires from user to put class member variable iDbgLevel
    into their class. This variable is used the same way as the static flag by using bit-mask to define enabled traces. See
    the macro definitions for details.
*/


#if defined(_DEBUG)
#define _DBG_MUST_BREAK
#else
#define _DBG_MUST_BREAK
#endif

/** Critical trace-level is used when system is about to down very soon because of critical error. 
    In most cases this trace level can be replaced by using assert with trace but in some cases
    more information can be provided by using this debug level
*/
#define CRIT_LEVEL              0x00000001

/** Serious trace-level is used when something bad and unexpected has happened but system might be
    able to recover. In another words, software is not going to bring system forcefully down but
    that's exactly what might happen due to an error.
*/
#define SERIOUS_LEVEL           0x00000002

/* Error level is used to trace various errors, which are due to legal errors in normal operation. */
#define ERROR_LEVEL             0x00000004

/* Warning level is used to trace various warning, which are due to abnormal behaviour. */
#define WARNING_LEVEL           0x00000008

/* Info level is used to trace all general information. */
#define INFO_LEVEL              0x00000010

/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_1          0x000010000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_2          0x000020000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_3          0x000040000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_4          0x000080000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_5          0x000100000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_6          0x000200000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_7          0x000400000
/* User definable trace level. This comment and definition should be replaced by the real usage. */
#define USER_DEFINED_8          0x000800000

#define DEVICE_1_MASK           0x0000f000
#define DEVICE_2_MASK           0x000f0000
#define DEVICE_3_MASK           0x00f00000
#define DEVICE_4_MASK           0x0f000000

// Override debug-level definition for your own in MMP file
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0x000000FF
#endif



#pragma warning(disable : 4127)    // conditional expression is constant

#if defined(_DEBUG) || defined(TRACES)

// Do not directly use this macro
//lint -emacro({717}, UtilDump)   do while(0)
//lint -emacro({774}, UtilDump)   conditional expression is constant
#define UtilDump(PFUNCTION,SRCLEVEL,TRGLEVEL,STRING) \
        do { \
            TUint32 _level = (TRGLEVEL); \
            if (SRCLEVEL & _level) { \
            PFUNCTION STRING; \
            } \
        } while (0)

#ifdef __KERNEL_MODE__
	#define TraceDump(LEVEL,STRING) UtilDump(Kern::Printf,DEBUG_LEVEL,LEVEL,STRING)
	#define RTraceDump(LEVEL,STRING) UtilDump(Kern::Printf,iDbgLevel,LEVEL,STRING)
#else
	#define TraceDump(LEVEL,STRING) UtilDump(RDebug::Print,DEBUG_LEVEL,LEVEL,STRING)
	#define RTraceDump(LEVEL,STRING) UtilDump(RDebug::Print,iDbgLevel,LEVEL,STRING)
#endif

#else  // it is release mode
//lint -emacro({717}, TraceDump) do while(0)
#define TraceDump(LEVEL,STRING) do {} while (0)
//lint -emacro({717}, RTraceDump) do while(0)
#define RTraceDump(LEVEL,STRING) do {} while (0)

#endif // _DEBUG




#endif // GENDEBUG_H

// End of file

