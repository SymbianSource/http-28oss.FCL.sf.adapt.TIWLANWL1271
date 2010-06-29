/*
 * OsApiDepend.h
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


/*--------------------------------------------------------------------------*/
/* Module:      OsApiDepend.h*/
/**/
/* Purpose:     This module defines unified interface to the OS specific*/
/*              sources and services.*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef __OS_API_DEPEND_H__
#define __OS_API_DEPEND_H__

/** \file  OsApiDepend.h 
 * \brief Operating System APIs that depend OS\n
 * This module defines unified interface to the OS, that different between the OSs, the specific sources and services
 */

#include "tidef.h"

/** \brief  OS  IRQ Serviced
 * 
 * \param  OsContext 	- Handle to the OS object
 * \return void
 * 
 * \par Description
 * This function is used in Level IRQ only. At this point the interrupt line is not asserted anymore
 * and we can inform the OS to enable IRQ again.
 * 
 * \sa
 */ 
void os_InterruptServiced (TI_HANDLE OsContext);

/** \brief  OS Protect Lock
 * 
 * \param  OsContext        - Handle to the OS object
 * \param  ProtectContext   - Handle to the mutex/spin lock object
 * \return void
 * 
 * \par Description
 * This function locks the mutex/spin lock object. E.g. the caller acquires a mutex/spin lock and gains exclusive
 * access to the shared resources, that the mutex/spin lock protects of.
 *
 * \sa
 */
#define OS_ENTER_CRITICAL_SECTION(OsContext, ProtectContext)


/** \brief  OS Protect Unlock
 * 
 * \param  OsContext        - Handle to the OS object
 * \param  ProtectContext   - Handle to the mutex/spin lock object
 * \return void
 * 
 * \par Description
 * This function unlocks the mutex/spin lock object.
 * 
 * \sa
 */
#define OS_LEAVE_CRITICAL_SECTION(OsContext, ProtectContext)


/** \brief  OS Schedule Request
 * 
 * \param  OsContext    - Handle to the OS object
 * \return TI_NOK (1) that indicate to the DR to call the CB by himself 
 * 
 * \par Description
 * in Symbian we dont have scheduling support
 * 
 * \sa
 */
#define OS_REQUEST_SCHEDULE(OsContext) \
    TI_NOK

/** \brief  OS  IRQ Serviced
 * 
 * \param  OsContext    - Handle to the OS object
 * \return void
 * 
 * \par Description
 * This function is used in Level IRQ only. At this point the interrupt line is not asserted anymore
 * and we can inform the OS to enable IRQ again.
 * 
 * \sa
 */ 
 /* Mark that interrupt was serviced */
#define OS_INTERRUPT_SERVICED(OsContext) \
	os_InterruptServiced (OsContext)

#endif
