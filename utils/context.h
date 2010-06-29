/*
 * context.h
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



/** \file   context.h 
 *  \brief  context module header file.                                  
 *
 *  \see    context.c
 */

#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#define MAX_CLIENTS     8   /* Maximum number of clients using context services */
#define MAX_NAME_SIZE   16  /* Maximum client's name string size */

#ifdef TI_DBG
typedef struct 
{
    TI_UINT32       uSize;                  /* Clients' name string size */
    char            sName [MAX_NAME_SIZE];  /* Clients' name string      */
} TClientName;	
#endif /* TI_DBG */

/* The callback function type for context clients */
typedef void (*TContextCbFunc)(TI_HANDLE hCbHndl);

/* context module structure */
typedef struct 
{
	TI_HANDLE        hOs;
	TI_HANDLE        hReport; 

    TI_HANDLE        hProtectionLock;              /* Handle of protection lock used by context clients */
    TI_UINT32        uNumClients;                  /* Number of registered clients      */
    TContextCbFunc   aClientCbFunc [MAX_CLIENTS];  /* Clients' callback functions       */
    TI_HANDLE        aClientCbHndl [MAX_CLIENTS];  /* Clients' callback handles         */
    TI_BOOL          aClientEnabled[MAX_CLIENTS];  /* Clients' enable/disable flags     */
    TI_BOOL          aClientPending[MAX_CLIENTS];  /* Clients' pending flags            */

#ifdef TI_DBG
    TClientName      aClientName   [MAX_CLIENTS];  /* Clients' name string              */
    TI_UINT32        aRequestCount [MAX_CLIENTS];  /* Clients' schedule requests counter*/
    TI_UINT32        aInvokeCount  [MAX_CLIENTS];  /* Clients' invocations counter      */
#endif

} TContext;	


/* Macros */
/* =======*/
/** 
 * \fn     CONTEXT_ENTER_CRITICAL_SECTION / CONTEXT_LEAVE_CRITICAL_SECTION
 * \brief  Lock / Unlock context related critical sections
 * 
 * The context clients should use these macros for protecting their critical sections
 *   when handling context transition to driver context.
 * 
 * \note   
 * \param  hContext   - The module handle
 * \return void 
 * \sa     
 */ 
	/* Start critical section protection */ 
#define CONTEXT_ENTER_CRITICAL_SECTION(hContext) \
	OS_ENTER_CRITICAL_SECTION(((TContext *)hContext)->hOs, ((TContext *)hContext)->hProtectionLock);

	/* Stop critical section protection */ 
#define CONTEXT_LEAVE_CRITICAL_SECTION(hContext) \
	OS_LEAVE_CRITICAL_SECTION(((TContext *)hContext)->hOs, ((TContext *)hContext)->hProtectionLock);


/* External Functions Prototypes */
/* ============================= */
TI_HANDLE context_Create          (TI_HANDLE hOs);
TI_STATUS context_Destroy         (TI_HANDLE hContext);
void      context_Init            (TI_HANDLE hContext, TI_HANDLE hOs, TI_HANDLE hReport);

TI_UINT32 context_RegisterClient (TI_HANDLE       hContext,
                                  TContextCbFunc  fCbFunc,
                                  TI_HANDLE       hCbHndl,
                                  TI_BOOL         bEnable,
                                  char           *sName,
                                  TI_UINT32       uNameSize);

void      context_RequestSchedule (TI_HANDLE hContext, TI_UINT32 uClientId);
void      context_DriverTask      (TI_HANDLE hContext);
void      context_EnableClient    (TI_HANDLE hContext, TI_UINT32 uClientId);
void      context_DisableClient   (TI_HANDLE hContext, TI_UINT32 uClientId);
void      context_DisableClient   (TI_HANDLE hContext, TI_UINT32 uClientId);
void      context_EnableClient    (TI_HANDLE hContext, TI_UINT32 uClientId);
#ifdef TI_DBG
void      context_Print           (TI_HANDLE hContext);
#endif /* TI_DBG */



#endif  /* _CONTEXT_H_ */


