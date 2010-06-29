/*
 * FwEvent.c
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


/** \file  FwEvent.c
 *  \brief Handle firmware events
 * 
 *   
 * \par Description
 *      Call the appropriate event handler.
 *
 *  \see FwEvent.h
 */

#define __FILE_ID__  FILE_ID_104
#include "tidef.h"
#include "report.h"
#include "context.h"
#include "osApi.h"
#include "TWDriver.h"
#include "TWDriverInternal.h"
#include "FwEvent.h" 
#include "txResult_api.h"
#include "CmdMBox_api.h"
#include "rxXfer_api.h" 
#include "txXfer_api.h" 
#include "txHwQueue_api.h"
#include "eventMbox_api.h"
#include "TwIf.h"
#ifdef TI_DBG
    #include "tracebuf_api.h"
#endif

#ifdef _VLCT_
extern int trigger_another_read;
#endif

#define USE_SDIO_24M_WORKAROUND
#define FW_STATUS_MEM_ADDRESS   0x40400


#define TXN_FW_EVENT_SET_MASK_ADDR(pFwEvent)    pFwEvent->tMaskTxn.tTxnStruct.uHwAddr = HINT_MASK;
#define TXN_FW_EVENT_SET_UNMASK_ADDR(pFwEvent)  pFwEvent->tUnMaskTxn.tTxnStruct.uHwAddr = HINT_MASK;
#define TXN_FW_EVENT_SET_STATUS_ADDR(pFwEvent)  pFwEvent->tFwStatusTxn.tTxnStruct.uHwAddr = ACX_REG_INTERRUPT_CLEAR;
#define TXN_FW_EVENT_SET_FW_MEM_ADDR(pFwEvent)  pFwEvent->tMemFwStatusTxn.tTxnStruct.uHwAddr = FW_STATUS_MEM_ADDRESS;
/********************* static function declerations *************************/

/*
 * \brief	Call FwEvent client's event handler
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * 
 * \sa fwEvent_ReadCompleteCb
 */
static void fwEvent_CallHandler (TI_HANDLE hFwEvent);


/*
 * \brief	Create the FwEvent module object
 * 
 * \param  hOs  - OS module object handle
 * \return Handle to the created object
 * 
 * \par Description
 * Calling this function creates a FwEvent object
 * 
 * \sa fwEvent_Destroy
 */
TI_HANDLE fwEvent_Create (TI_HANDLE hOs)
{
    TfwEvent *pFwEvent;

    pFwEvent = os_memoryAlloc (hOs, sizeof(TfwEvent),MemoryNormal);
    if (pFwEvent == NULL)
    {
        return NULL;
    }

    os_memoryZero (hOs, pFwEvent, sizeof(TfwEvent));

    /* Allocation of Data */
    pFwEvent->tMaskTxn.pData = os_memoryAlloc (hOs, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ,MemoryDMA);
    if (pFwEvent->tMaskTxn.pData == NULL) 
    {
        return NULL;
    }
    os_memoryZero (hOs, pFwEvent->tMaskTxn.pData, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ);
    pFwEvent->tMaskTxn.pData += WSPI_PAD_LEN_READ;

    /* Allocation of Data */
    pFwEvent->tUnMaskTxn.pData = os_memoryAlloc (hOs, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ,MemoryDMA);
    if (pFwEvent->tUnMaskTxn.pData == NULL) 
    {
        return NULL;
    }
    os_memoryZero (hOs, pFwEvent->tUnMaskTxn.pData, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ);
    pFwEvent->tUnMaskTxn.pData += WSPI_PAD_LEN_READ;

     /* Allocation of FW Status buffer */
    pFwEvent->tFwStatusTxn.pFwStatus = os_memoryAlloc (hOs, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ,MemoryDMA);
    if (pFwEvent->tFwStatusTxn.pFwStatus == NULL) 
    {
        return NULL;
    }
    os_memoryZero (hOs, pFwEvent->tFwStatusTxn.pFwStatus, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ);
    pFwEvent->tFwStatusTxn.pFwStatus += WSPI_PAD_LEN_READ;

     /* Allocation of FW Status buffer */
    pFwEvent->tMemFwStatusTxn.pFwStatus = os_memoryAlloc (hOs, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ,MemoryDMA);
    if (pFwEvent->tMemFwStatusTxn.pFwStatus == NULL) 
    {
        return NULL;
    }
    os_memoryZero (hOs, pFwEvent->tMemFwStatusTxn.pFwStatus, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ);
    pFwEvent->tMemFwStatusTxn.pFwStatus += WSPI_PAD_LEN_READ;


    pFwEvent->hOs = hOs;

    return (TI_HANDLE)pFwEvent;
}


/*
 * \brief	Destroys the FwEvent object
 * 
 * \param  hFwEvent  - The object to free
 * \return TI_OK
 * 
 * \par Description
 * Calling this function destroys a FwEvent object
 * 
 * \sa fwEvent_Create
 */
TI_STATUS fwEvent_Destroy (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    if (pFwEvent->tMaskTxn.pData)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent->tMaskTxn.pData - WSPI_PAD_LEN_READ, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ);
    }

    if (pFwEvent->tUnMaskTxn.pData)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent->tUnMaskTxn.pData - WSPI_PAD_LEN_READ, sizeof (TI_UINT32) + WSPI_PAD_LEN_READ);
    }

    if (pFwEvent->tFwStatusTxn.pFwStatus)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent->tFwStatusTxn.pFwStatus - WSPI_PAD_LEN_READ, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ);
    }

    if (pFwEvent->tMemFwStatusTxn.pFwStatus)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent->tMemFwStatusTxn.pFwStatus - WSPI_PAD_LEN_READ, sizeof (FwStatus_t) + WSPI_PAD_LEN_READ);
    }

    if (pFwEvent)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent, sizeof(TfwEvent));
    }

    return TI_OK;
}


/*
 * \brief	Config the FwEvent module object
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \param  hTWD  - Handle to TWD module
 * \return TI_OK
 * 
 * \par Description
 * From hTWD we extract : hOs, hReport, hTwIf, hContext,
 *      hHealthMonitor, hEventMbox, hCmdMbox, hRxXfer, 
 *      hTxHwQueue, hTxResult
 * In this function we also register the FwEvent to the context engine
 * 
 * \sa
 */
TI_STATUS fwEvent_Init (TI_HANDLE hFwEvent, TI_HANDLE hTWD)
{
    TfwEvent  *pFwEvent = (TfwEvent *)hFwEvent;
    TTwd      *pTWD = (TTwd *)hTWD;
    TTxnStruct* pTxn;
    
    pFwEvent->hTWD              = hTWD;
    pFwEvent->hOs               					= pTWD->hOs;
    pFwEvent->hReport           					= pTWD->hReport;
    pFwEvent->hContext          					= pTWD->hContext;
    pFwEvent->hTwIf             = pTWD->hTwIf;
    pFwEvent->hHealthMonitor    = pTWD->hHealthMonitor;
    pFwEvent->hEventMbox        = pTWD->hEventMbox;
    pFwEvent->hCmdMbox          = pTWD->hCmdMbox;
    pFwEvent->hRxXfer           = pTWD->hRxXfer;
    pFwEvent->hTxHwQueue        = pTWD->hTxHwQueue;
    pFwEvent->hTxXfer           = pTWD->hTxXfer;
    pFwEvent->hTxResult         = pTWD->hTxResult;

    pFwEvent->eFwEventState       = FW_EVENT_STATE_IDLE;
    pFwEvent->uEventMask          = 0;
    pFwEvent->uEventVector        = 0;
    pFwEvent->bFwNotificationFlag = TI_FALSE;

    pTxn = (TTxnStruct*)&pFwEvent->tMaskTxn.tTxnStruct;
    TXN_PARAM_SET(pTxn, TXN_HIGH_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, HINT_MASK, pFwEvent->tMaskTxn.pData, REGISTER_SIZE, NULL, NULL)

    pTxn = (TTxnStruct*)&pFwEvent->tUnMaskTxn.tTxnStruct;
    TXN_PARAM_SET(pTxn, TXN_HIGH_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, HINT_MASK, pFwEvent->tUnMaskTxn.pData, REGISTER_SIZE, NULL, NULL)

#ifdef USE_SDIO_24M_WORKAROUND /* FW_STATUS moved from registers to memory area */
    /* First txn is HINT_STT_CLR register */
    pTxn = (TTxnStruct*)&pFwEvent->tFwStatusTxn.tTxnStruct;
    TXN_PARAM_SET(pTxn, TXN_HIGH_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, ACX_REG_INTERRUPT_CLEAR, pFwEvent->tFwStatusTxn.pFwStatus, REGISTER_SIZE, NULL, NULL)
    /* FW status from memory area */
    pTxn = (TTxnStruct*)&pFwEvent->tMemFwStatusTxn.tTxnStruct;
    TXN_PARAM_SET(pTxn, TXN_HIGH_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, FW_STATUS_MEM_ADDRESS, pFwEvent->tMemFwStatusTxn.pFwStatus, sizeof(FwStatus_t)-REGISTER_SIZE, (TTxnDoneCb)fwEvent_ReadCompleteCb, hFwEvent)
#else
    pTxn = (TTxnStruct*)&pFwEvent->tFwStatusTxn.tTxnStruct;
    TXN_PARAM_SET(pTxn, TXN_HIGH_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, ACX_REG_INTERRUPT_CLEAR, pFwEvent->tFwStatusTxn.pFwStatus, sizeof(FwStatus_t), (TTxnDoneCb)fwEvent_ReadCompleteCb, hFwEvent)
#endif
    /* 
     *  Register the FwEvent to the context engine and get the client ID.
     *  The FwEvent() will be called from the context_DriverTask() after scheduled
     *    by a FW-Interrupt (see fwEvent_InterruptRequest()).
     */
    pFwEvent->uContextId = context_RegisterClient (pFwEvent->hContext,
                                                   fwEvent_Handle,
                                                   hFwEvent,
                                                   TI_FALSE,
                                                   "FW_EVENT",
                                                   sizeof("FW_EVENT"));
	
    return TI_OK;
}


/*
 * \brief	Call FwEvent client's event handler
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * 
 * \sa fwEvent_ReadCompleteCb
 */
static void fwEvent_CallHandler (TI_HANDLE hFwEvent)
{
    TfwEvent   *pFwEvent = (TfwEvent *)hFwEvent;

    if (pFwEvent->uEventVector & ACX_INTR_WATCHDOG)
    {
        /* Fw watchdog timeout has occured */
        TWD_WdExpireEvent (pFwEvent->hTWD);
    }

    if (pFwEvent->uEventVector & ACX_INTR_INIT_COMPLETE)
    {
        TRACE0(pFwEvent->hReport, REPORT_SEVERITY_INFORMATION, "fwEvent_CallHandler: INIT_COMPLETE\n");
    }
	/* Change to handle the command MBOX before the event MBOX to maintain order for WHA command response
	 * and follow command complete
	 */
    if (pFwEvent->uEventVector & ACX_INTR_CMD_COMPLETE)
    {
        /* Command Mbox completed */
        cmdMbox_CommandComplete(pFwEvent->hCmdMbox);
    }
    if (pFwEvent->uEventVector & ACX_INTR_EVENT_A)
    {
        eventMbox_Handle(pFwEvent->hEventMbox,(FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus));
    }
    if (pFwEvent->uEventVector & ACX_INTR_EVENT_B)
    {
        eventMbox_Handle(pFwEvent->hEventMbox,(FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus));
    }
    

    /* The DATA interrupt is shared by all data path events, so call all Tx and Rx clients */
    if (pFwEvent->uEventVector & ACX_INTR_DATA)
    {
        rxXfer_RxEvent (pFwEvent->hRxXfer, (FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus));

        txHwQueue_UpdateFreeResources (pFwEvent->hTxHwQueue, (FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus));

        txResult_TxCmpltIntrCb (pFwEvent->hTxResult, (FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus));
    } 

    /* After handling all raised bits, we can negate them */
    ((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->intrStatus &= pFwEvent->uEventMask;
}


/*
 * \brief	Requests the context engine to schedule the driver task
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * Called by the FW-Interrupt ISR.
 * Requests the context engine to schedule the driver task 
 * for handling the FW-Events (FwEvent callback).
 * 
 * \sa
 */
void fwEvent_InterruptRequest (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    /* Request switch to driver context for handling the FW-Interrupt event */
    context_RequestSchedule (pFwEvent->hContext, pFwEvent->uContextId);
}


/*
 * \brief	Handle the FW interrupts
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * Called from context module upon receiving FW interrupt
 * The function mask the interrupts and reads the FW status
 * 
 * \sa
 */

void fwEvent_Handle (TI_HANDLE hFwEvent)
{
    TfwEvent   *pFwEvent = (TfwEvent *)hFwEvent;
    ETxnStatus rc;

    /* NOTE: pFwEvent may be uninitialized at init stage */
    if (pFwEvent != NULL)
    {
        if (pFwEvent->eFwEventState != FW_EVENT_STATE_IDLE)
        {
	    os_InterruptServiced (pFwEvent->hOs);
            twIf_HwAvailable(pFwEvent->hTwIf);
            return;
        }

        pFwEvent->eFwEventState = FW_EVENT_STATE_READING;
        pFwEvent->bFwNotificationFlag = TI_TRUE;

        twIf_Awake(pFwEvent->hTwIf);
        twIf_HwAvailable(pFwEvent->hTwIf);

        /* Write HINT mask */
        *((TI_UINT32*)(pFwEvent->tMaskTxn.pData)) = ACX_INTR_ALL;
        TXN_FW_EVENT_SET_MASK_ADDR(pFwEvent)
        twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tMaskTxn.tTxnStruct));

#ifdef USE_SDIO_24M_WORKAROUND /* FW_STATUS moved from registers to memory area */
        /* 
         * Read first register (HINT_STT_CLR) only from registers area
         */
        TXN_FW_EVENT_SET_STATUS_ADDR(pFwEvent)
        twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tFwStatusTxn.tTxnStruct));

        /* 
         * Read other 16 registers value from memory area FW_STATUS_MEM_ADDRESS
         */
        TXN_FW_EVENT_SET_FW_MEM_ADDR(pFwEvent)
        rc = twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tMemFwStatusTxn.tTxnStruct));

        if (rc == TXN_STATUS_COMPLETE)
        {
            fwEvent_ReadCompleteCb(hFwEvent);
        }
#else

        /* Read the Fw status */
        TXN_FW_EVENT_SET_STATUS_ADDR(pFwEvent)
        rc = twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tFwStatusTxn.tTxnStruct));

        if (rc == TXN_STATUS_COMPLETE)
        {
            fwEvent_ReadCompleteCb(hFwEvent);
        }
#endif
    }
    else
    {
        os_InterruptServiced (pFwEvent->hOs);

    /* end of if */
    }
}


/*
 * \brief	Handle the Fw Status information 
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * This function is called from fwEvent_Handle on a sync read, or from TwIf as a CB on an async read.
 * It calls fwEvent_CallHandler to handle the triggered interrupts.
 * 
 * \sa fwEvent_Handle
 */
void fwEvent_ReadCompleteCb (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;
    
#ifdef USE_SDIO_24M_WORKAROUND /* FW_STATUS moved from registers to memory area */
    /* 
     * copy FW status 16 registers values from memory transaction read area to the original place 
*/
    os_memoryCopy (pFwEvent->hOs, &((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->counters, &((FwStatus_t*)(pFwEvent->tMemFwStatusTxn.pFwStatus))->counters, sizeof(FwStatus_t)-REGISTER_SIZE);
#endif
    
    os_InterruptServiced (pFwEvent->hOs);
	  
    /* If we were called because of an interrupt */
	if (pFwEvent->bFwNotificationFlag)
    {
        /* In case of level interrupt we need to clear the line */
        /*os_InterruptServiced(pFwEvent->hOs);*/

        /*
         * Sync to fw time so we can update the tx packets
         * on the delta time that they spent in the driver 
         */
        pFwEvent->uFwTimeOffset = (os_timeStampMs (pFwEvent->hOs) * 1000) - 
                                  ENDIAN_HANDLE_LONG (((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->fwLocalTime);

        pFwEvent->bFwNotificationFlag = TI_FALSE;
    }

    pFwEvent->uEventVector = ((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->intrStatus;

    /* 
     * Mask unwanted interrupts. 
     */
    pFwEvent->uEventVector &= pFwEvent->uEventMask;

    fwEvent_CallHandler(hFwEvent);

    /* Check if the state is changed in the context of the event callbacks */
    if (pFwEvent->eFwEventState == FW_EVENT_STATE_IDLE)
    {
        /*
         * When fwEvent_stop is called state is changed to IDLE
         * This is done in the context of the above events callbacks
         * Don't send the UNMASK transaction because the driver stop process includes power off
         */ 
        TRACE0(pFwEvent->hReport, REPORT_SEVERITY_WARNING, "fwEvent_ReadCompleteCb : State is IDLE ! don't send the UNMASK");
        return;
    }

    /* Write HINT unmask */
    *((TI_UINT32*)(pFwEvent->tUnMaskTxn.pData)) = ~pFwEvent->uEventMask;
    TXN_FW_EVENT_SET_UNMASK_ADDR(pFwEvent)
    twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tUnMaskTxn.tTxnStruct));

    twIf_Sleep(pFwEvent->hTwIf);
    pFwEvent->eFwEventState = FW_EVENT_STATE_IDLE;
} 


/*
 * \brief	Translate host to FW time (Usec)
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \param  uHostTime - The host time in MS to translate
 *
 * \return FW Time in Usec
 * 
 * \par Description
 * 
 * \sa
 */
TI_UINT32 fwEvent_TranslateToFwTime (TI_HANDLE hFwEvent, TI_UINT32 uHostTime)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    return ((uHostTime * 1000) - pFwEvent->uFwTimeOffset);
}


/*
 * \brief	Unmask only cmd-cmplt and events interrupts (needed for init phase)
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return Event mask
 * 
 * \par Description
 * Unmask only cmd-cmplt and events interrupts (needed for init phase)
 *                  and return interrupt enabled bit mask.
 * 
 * \sa
 */
TI_UINT32 fwEvent_GetInitMask (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    /* Unmask only the interrupts needed for the FW configuration process. */
    pFwEvent->uEventMask = ACX_INTR_CMD_COMPLETE | ACX_INTR_EVENT_A | ACX_INTR_EVENT_B;

    return pFwEvent->uEventMask;
}


/*
 * \brief	Stop & reset FwEvent (called by the driver stop process)
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return TI_OK
 * 
 * \par Description
 *
 * \sa
 */
TI_STATUS fwEvent_Stop (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    pFwEvent->eFwEventState = FW_EVENT_STATE_IDLE;
    ((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->intrStatus = 0;
    pFwEvent->uEventMask = 0;
    pFwEvent->bFwNotificationFlag = TI_FALSE;
    pFwEvent->uEventVector = 0;    
    
    return TI_OK;
}


/*
 * \brief	Unmask all interrupts, set Rx interrupt bit and call FwEvent_Handle
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * Called when driver Start or recovery process is completed.
 *              Unmask all interrupts, set Rx interrupt bit and call FwEvent_Handle 
 *                  (in case we missed an Rx interrupt in a recovery process).
 * 
 * \sa
 */
void fwEvent_EnableExternalEvents (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;

    /* Unmask all interrupts */
    pFwEvent->uEventMask    = ALL_EVENTS_VECTOR;

    /* Set Rx interrupt bit to invoke it (in case we missed it in a recovery/start process) */
    ((FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus))->intrStatus |= ACX_INTR_DATA;

    /* If in IDLE state, handle interrupts including the Rx we've just set manually */
    if (pFwEvent->eFwEventState == FW_EVENT_STATE_IDLE)
    {        
        fwEvent_GetFwStatus (hFwEvent);
    }
}


/*
 * \brief	Disable the FwEvent client of the context thread handler
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 *
 * \sa
 */
void fwEvent_DisableInterrupts(TI_HANDLE hFwEvent)
{
    TfwEvent  *pFwEvent = (TfwEvent *)hFwEvent;

    context_DisableClient (pFwEvent->hContext,pFwEvent->uContextId);
	
}


/*
 * \brief	Enable the FwEvent client of the context thread handler
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 *
 * \sa
 */
void fwEvent_EnableInterrupts(TI_HANDLE hFwEvent)
{
    TfwEvent  *pFwEvent = (TfwEvent *)hFwEvent;

    context_EnableClient (pFwEvent->hContext,pFwEvent->uContextId);
	
}


/*
 * \brief	Issue a FW status read (Not in response to any FW interrupt)
 * 
 * \param  hFwEvent  - FwEvent Driver handle
 * \return void
 * 
 * \par Description
 * Issue a FW status read (Not in response to any FW interrupt)
 * Called also from fwEvent_EnableExternalEvents after recovery.
 * 
 * \sa fwEvent_EnableExternalEvents
 */
void fwEvent_GetFwStatus (TI_HANDLE hFwEvent)
{
    TfwEvent   *pFwEvent = (TfwEvent *)hFwEvent;
    ETxnStatus rc;

    /* NOTE: pFwEvent may be uninitialized at init stage */
    if (pFwEvent != NULL)
    {
        if (pFwEvent->eFwEventState != FW_EVENT_STATE_IDLE)
        {
            twIf_HwAvailable(pFwEvent->hTwIf);
            return;
        }

        pFwEvent->eFwEventState = FW_EVENT_STATE_READING;
        
        twIf_Awake(pFwEvent->hTwIf);

        /* Write HINT mask */
        *((TI_UINT32*)(pFwEvent->tMaskTxn.pData)) = ACX_INTR_ALL;
        TXN_FW_EVENT_SET_MASK_ADDR(pFwEvent)
        twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tMaskTxn.tTxnStruct));

#ifdef USE_SDIO_24M_WORKAROUND /* FW_STATUS moved from registers to memory area */
        /* 
         * Read first register (HINT_STT_CLR) only from registers area
         */
        TXN_FW_EVENT_SET_STATUS_ADDR(pFwEvent)
        twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tFwStatusTxn.tTxnStruct));

        /* 
         * Read other 16 registers value from memory area 0x40200
         */
        TXN_FW_EVENT_SET_FW_MEM_ADDR(pFwEvent)
        rc = twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tMemFwStatusTxn.tTxnStruct));

        if (rc == TXN_STATUS_COMPLETE)
        {
            fwEvent_ReadCompleteCb(hFwEvent);
        }
#else
        /* Read the Fw status */
        TXN_FW_EVENT_SET_STATUS_ADDR(pFwEvent)
        rc = twIf_Transact(pFwEvent->hTwIf, &(pFwEvent->tFwStatusTxn.tTxnStruct));

        if (rc == TXN_STATUS_COMPLETE)
        {
            fwEvent_ReadCompleteCb(hFwEvent);
        }
#endif
    /* end of if */
    }
}


#ifdef TI_DBG

void fwEvent_PrintStat (TI_HANDLE hFwEvent)
{
    TfwEvent *pFwEvent = (TfwEvent *)hFwEvent;
    FwStatus_t *fwStat = (FwStatus_t*)(pFwEvent->tFwStatusTxn.pFwStatus); 
	int i;

    WLAN_OS_REPORT(("Print FW event module info\n"));
    WLAN_OS_REPORT(("==========================\n"));
    WLAN_OS_REPORT(("intrStatus = 0x%08x\n", fwStat->intrStatus));
    WLAN_OS_REPORT(("counters   = 0x%08x\n", fwStat->counters));
	for (i = 0; i < NUM_RX_PKT_DESC; i++)
    {
		WLAN_OS_REPORT(("rxPktsDesc[%1d] = 0x%08x\n", i, fwStat->rxPktsDesc[i]));
    }
	for (i = 0; i < NUM_TX_QUEUES; i++)
    {
		WLAN_OS_REPORT(("txReleasedBlks[%1d] = 0x%08x\n", i, fwStat->txReleasedBlks[i]));
    }
    WLAN_OS_REPORT(("fwLocalTime = 0x%08x\n", fwStat->fwLocalTime));
}

#endif  /* TI_DBG */

