/*
 * context.c
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


/** \file   context.c 
 *  \brief  The Context-Engine is an OS independent module, which provides the
 *            infrustracture for switching from external contexts to the driver's context.
 *          This includes also the driver task itself , which invokes the 
 *            driver specific handlers in the driver's context.
 *          The OS specific implementation under this module (e.g. task-switching or 
 *            protection-locking) is provided by the OS abstraction layer (osapi.c).
 * 
 *  \see    context.h, osapi.c
 */


#define __FILE_ID__  FILE_ID_125
#include "osApi.h"
#include "report.h"
#include "context.h"
#include "TWDriver.h"


/** 
 * \fn     context_Create 
 * \brief  Create the module 
 * 
 * Allocate and clear the module object.
 * 
 * \note    
 * \param  hOs      - Handle to Os Abstraction Layer
 * \return Handle of the allocated object 
 * \sa     context_Destroy
 */ 
TI_HANDLE context_Create (TI_HANDLE hOs)
{
	TI_HANDLE hContext;

	/* allocate module object */
	hContext = os_memoryAlloc (hOs, sizeof(TContext),MemoryNormal);
	
	if (!hContext)
	{
		WLAN_OS_REPORT (("context_Create():  Allocation failed!!\n"));
		return NULL;
	}
	
    os_memoryZero (hOs, hContext, (sizeof(TContext)));

	return (hContext);
}


/** 
 * \fn     context_Destroy
 * \brief  Destroy the module. 
 * 
 * Free the module memory.
 * 
 * \note   
 * \param  hContext - The module object
 * \return TI_OK on success or TI_NOK on failure 
 * \sa     context_Create
 */ 
TI_STATUS context_Destroy (TI_HANDLE hContext)
{
    TContext *pContext = (TContext *)hContext;

    /* Destroy the protection handle */
    os_protectDestroy (pContext->hOs, pContext->hProtectionLock);

    /* free module object */
	os_memoryFree (pContext->hOs, pContext, sizeof(TContext));
	
    return TI_OK;
}


/** 
 * \fn     context_Init 
 * \brief  Init required handles 
 * 
 * Init required handles.
 * 
 * \note    
 * \param  hContext  - The queue object
 * \param  hOs       - Handle to Os Abstraction Layer
 * \param  hReport   - Handle to report module
 * \return void        
 * \sa     
 */ 
void context_Init (TI_HANDLE hContext, TI_HANDLE hOs, TI_HANDLE hReport)
{
	TContext *pContext = (TContext *)hContext;

    pContext->hOs     = hOs;
    pContext->hReport = hReport;
	
    /* Create the module's protection lock and save its handle */
    pContext->hProtectionLock = os_protectCreate (pContext->hOs);
}


/** 
 * \fn     context_RegisterClient
 * \brief  Save client's parameters
 * 
 * This function is used by the Context clients in their init process, for registering 
 *   their information, 
 * This includes their callback function that should be invoked from the driver context
 *   when they are pending. 
 * 
 * \note   
 * \param  hContext - The module handle
 * \param  fCbFunc  - The client's callback function.
 * \param  hCbHndl  - The client's callback function handle.
 * \param  bEnable  - TRUE = Enabled.
 * \return TI_UINT32 - The index allocated for the client
 * \sa     
 */ 
TI_UINT32 context_RegisterClient (TI_HANDLE       hContext,
                                  TContextCbFunc  fCbFunc,
                                  TI_HANDLE       hCbHndl,
                                  TI_BOOL         bEnable,
                                  char           *sName,
                                  TI_UINT32       uNameSize)
{
	TContext *pContext = (TContext *)hContext;
    TI_UINT32 uClientId = pContext->uNumClients;

    /* If max number of clients is exceeded, report error and exit. */
    if (uClientId == MAX_CLIENTS) 
    {
        TRACE0(pContext->hReport, REPORT_SEVERITY_ERROR , "context_RegisterClient() MAX_CLIENTS limit exceeded!!\n");
        return 0;
    }

    /* Save the new client's parameters. */
    pContext->aClientCbFunc[uClientId]  = fCbFunc;
    pContext->aClientCbHndl[uClientId]  = hCbHndl;
    pContext->aClientEnabled[uClientId] = bEnable;
    pContext->aClientPending[uClientId] = TI_FALSE;

#ifdef TI_DBG
    if (uNameSize <= MAX_NAME_SIZE) 
    {
        os_memoryCopy(pContext->hOs, 
                      (void *)(pContext->aClientName[uClientId].sName),
                      (void *)sName, 
                      uNameSize);
        pContext->aClientName[uClientId].uSize = uNameSize;
    }
    else 
    {
        TRACE0(pContext->hReport, REPORT_SEVERITY_ERROR , "context_RegisterClient() MAX_NAME_SIZE limit exceeded!\n");
    }
#endif /* TI_DBG */

    /* Increment clients number and return new client ID. */
    pContext->uNumClients++;

    TRACE2(pContext->hReport, REPORT_SEVERITY_INIT , "context_RegisterClient(): Client=, ID=%d, enabled=%d\n", uClientId, bEnable);

    return uClientId;
}


/** 
 * \fn     context_RequestSchedule
 * \brief  Handle client's switch to driver's context.
 * 
 * This function is called by a client from external context event.
 * It sets the client's Pending flag and requests the driver's task scheduling.
 * Thus, the client's callback will be called afterwards from the driver context.
 * 
 * \note   
 * \param  hContext   - The module handle
 * \param  uClientId  - The client's index
 * \return void 
 * \sa     context_DriverTask
 */ 
void context_RequestSchedule (TI_HANDLE hContext, TI_UINT32 uClientId)
{
	TContext *pContext = (TContext *)hContext;

#ifdef TI_DBG
    pContext->aRequestCount[uClientId]++; 
    TRACE3(pContext->hReport, REPORT_SEVERITY_INFORMATION , "context_RequestSchedule(): Client=, ID=%d, enabled=%d, pending=%d\n", uClientId, pContext->aClientEnabled[uClientId], pContext->aClientPending[uClientId]);
#endif /* TI_DBG */

    /* Set client's Pending flag */
    pContext->aClientPending[uClientId] = TI_TRUE;

    /* 
     * If configured to switch context, request driver task scheduling.
     * Else (context switch not required) call the driver task directly.
	 * OS_REQUEST_SCHEDULE is OS depend macro, in case that the Os not support scheduling
	 * that macro should return TI_NOK.
     */
	if(TI_OK != OS_REQUEST_SCHEDULE(pContext->hOs))
    {
        context_DriverTask (hContext);
    }
}


/** 
 * \fn     context_DriverTask
 * \brief  The driver task
 * 
 * This function is the driver's main task that always runs in the driver's 
 * single context, scheduled through the OS . 
 * Only one instantiation of this task may run at a time!
 * 
 * \note   
 * \param  hContext   - The module handle
 * \return void 
 * \sa     context_RequestSchedule
 */ 
void context_DriverTask (TI_HANDLE hContext)
{
	TContext       *pContext = (TContext *)hContext;
    TContextCbFunc  fCbFunc;
    TI_HANDLE       hCbHndl;
    TI_UINT32       i;

    TRACE0(pContext->hReport, REPORT_SEVERITY_INFORMATION , "context_DriverTask():\n");

    /* For all registered clients do: */
	for (i = 0; i < pContext->uNumClients; i++)
    {
        /* If client is pending and enabled */
        if (pContext->aClientPending[i]  &&  pContext->aClientEnabled[i])
        {
            #ifdef TI_DBG
                pContext->aInvokeCount[i]++; 
                TRACE1(pContext->hReport, REPORT_SEVERITY_INFORMATION , "Invoking - Client=, ID=%d\n", i);
            #endif /* TI_DBG */

            /* Clear client's pending flag */
            pContext->aClientPending[i] = TI_FALSE;

            /* Call client's callback function */
            fCbFunc = pContext->aClientCbFunc[i];
            hCbHndl = pContext->aClientCbHndl[i];
            fCbFunc(hCbHndl);
        }
    }
}


/** 
 * \fn     context_EnableClient / context_DisableClient
 * \brief  Enable a specific client activation
 * 
 * Called by the driver main when needed to enble/disable a specific event handling.
 * The Enable function also schedules the driver-task if the specified client is pending.
 * 
 * \note   
 * \param  hContext   - The module handle
 * \param  uClientId  - The client's index
 * \return void 
 * \sa     context_DriverTask
 */ 
void context_EnableClient (TI_HANDLE hContext, TI_UINT32 uClientId)
{
	TContext *pContext = (TContext *)hContext;

#ifdef TI_DBG
    if (pContext->aClientEnabled[uClientId])
    {
        TRACE0(pContext->hReport, REPORT_SEVERITY_ERROR , "context_EnableClient() Client  already enabled!!\n");
        return;
    }
    TRACE3(pContext->hReport, REPORT_SEVERITY_INFORMATION , "context_EnableClient(): Client=, ID=%d, enabled=%d, pending=%d\n", uClientId, pContext->aClientEnabled[uClientId], pContext->aClientPending[uClientId]);
#endif /* TI_DBG */

    /* Enable client */
    pContext->aClientEnabled[uClientId] = TI_TRUE;

    /* If client is pending, schedule driver task */
    if (pContext->aClientPending[uClientId])
    {
        /* 
         * If configured to switch context, request driver task scheduling.
         * Else (context switch not required) call the driver task directly. 
         */
		if (TI_OK != OS_REQUEST_SCHEDULE(pContext->hOs)) 
        {
            context_DriverTask (hContext);
        }
    }
}

void context_DisableClient (TI_HANDLE hContext, TI_UINT32 uClientId)
{
	TContext *pContext = (TContext *)hContext;

#ifdef TI_DBG
    if (!pContext->aClientEnabled[uClientId])
    {
        TRACE0(pContext->hReport, REPORT_SEVERITY_ERROR , "context_DisableClient() Client  already disabled!!\n");
        return;
    }
    TRACE3(pContext->hReport, REPORT_SEVERITY_INFORMATION , "context_DisableClient(): Client=, ID=%d, enabled=%d, pending=%d\n", uClientId, pContext->aClientEnabled[uClientId], pContext->aClientPending[uClientId]);
#endif /* TI_DBG */

    /* Disable client */
    pContext->aClientEnabled[uClientId] = TI_FALSE;
}

/** 
 * \fn     context_Print
 * \brief  Print module information
 * 
 * Print the module's clients parameters.
 * 
 * \note   
 * \param  hContext - The queue object
 * \return void
 * \sa     
 */ 

#ifdef TI_DBG

void context_Print(TI_HANDLE hContext)
{
	TContext *pContext = (TContext *)hContext;
    TI_UINT32 i;

    WLAN_OS_REPORT(("context_Print():  %d Clients Registered:\n", pContext->uNumClients));
    WLAN_OS_REPORT(("=======================================\n"));

	for (i = 0; i < pContext->uNumClients; i++)
	{
		WLAN_OS_REPORT(("Client %d - %s: CbFunc=0x%x, CbHndl=0x%x, Enabled=%d, Pending=%d, Requests=%d, Invoked=%d\n",
                        i,
                        pContext->aClientName[i].sName,
                        pContext->aClientCbFunc[i],
                        pContext->aClientCbHndl[i],
                        pContext->aClientEnabled[i],
                        pContext->aClientPending[i],
                        pContext->aRequestCount[i],
                        pContext->aInvokeCount[i] ));
	}
}

#endif /* TI_DBG */


