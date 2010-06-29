/*
 * FwEvent.h
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


/** \file FwEvent.h
 *  \brief FwEvent internal defenitions
 *
 *  \see FwEvent.c
 */

#ifndef _FW_EVENT_
#define _FW_EVENT_

#include "FwEvent_api.h"
#include "TwIf.h"
#include "public_host_int.h"


#define ALL_EVENTS_VECTOR        ACX_INTR_WATCHDOG | ACX_INTR_INIT_COMPLETE | ACX_INTR_EVENT_A |\
                                 ACX_INTR_EVENT_B | ACX_INTR_CMD_COMPLETE |ACX_INTR_HW_AVAILABLE |\
                                 ACX_INTR_DATA

#define ALL_EVENTS_VECTOR_NEGATE 0xFFFFFFC0   


/** \enum EFwEventState
 * \brief FwEvent states
 * 
 * \par Description
 * 
 * \sa 
 */
typedef enum
{
    FW_EVENT_STATE_IDLE,    
    FW_EVENT_STATE_READING

} EFwEventState;


typedef struct 
{
    TTxnStruct              tTxnStruct;
    TI_UINT8                *pData; 

} TRegisterTxn;

typedef struct 
{
    TTxnStruct              tTxnStruct;
    TI_UINT8                *pFwStatus;

} TFwStatusTxn;


/** \struct TfwEvent
 * \brief FW event structure
 * 
 * \par Description
 * 
 * \sa	
 */ 
typedef struct 
{
    EFwEventState       eFwEventState;           	/* State machine state */
    TI_UINT32           uEventMask;              	/* Static interrupt event mask */
    TI_UINT32           uEventVector;
    TI_BOOL             bFwNotificationFlag;
    TRegisterTxn        tMaskTxn;
    TRegisterTxn        tUnMaskTxn;
    TFwStatusTxn        tFwStatusTxn;
    TFwStatusTxn        tMemFwStatusTxn;
    
    TI_UINT32           uFwTimeOffset;              /* Offset in microseconds between driver and FW clocks */

    TI_BOOL             bNotIntr;                   /* Boolean that sgnals us that the fwStatus read is not from an interrupt origin */

    TI_HANDLE           hOs;                    	/* OS handle */
    TI_HANDLE           hTWD;
    TI_HANDLE           hReport;                	/* Report handle */
    TI_HANDLE           hContext;               	/* Context-Engine handle */
    TI_UINT32           uContextId;             	/* The ID allocated on registration to context module */
    TI_HANDLE           hTwIf;                      /* TwIf handle */
    TI_HANDLE           hHealthMonitor;             /* healthMonitor handle */
    TI_HANDLE           hEventMbox;
    TI_HANDLE           hCmdMbox;
    TI_HANDLE           hRxXfer;
    TI_HANDLE           hTxXfer;
    TI_HANDLE           hTxHwQueue;
    TI_HANDLE           hTxResult;

} TfwEvent; 

#endif  /* _FW_EVENT_ */
        
